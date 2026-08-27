/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iRCInterface/RCInterface.h
   Last Ed: 2026-08-12
     Brief:
        MOOS app that owns the CRSF RC serial link in BOTH
        directions. Decodes RC channels (0x16) and link
        statistics (0x14) from an ELRS receiver and publishes
        RC_CH1..RC_CH16, RC_CONNECTED, RC_FRAME_VALID etc. per
        RC Contract v2 (docs/rc_handset_architecture.md, frozen
        2026-08-11): CH1-4 axes exactly as iRCReader scaled them,
        CH5-10/12 discrete channels guard-band decoded to the
        TX16S semantic map (KILL/MODE/ACTION/...), CH11 a thrust
        limit percent, plus the RC_LINK_* variables CRSF makes
        possible. Sends battery / GPS / flight-mode / distance
        telemetry back to the handset on the same port.

        Supersedes iRCReader (SBUS), which stays in the tree
        untouched as the transitional fallback: reverting is a
        launch-config change, not a rebuild.

        Design: docs/rc_crsf_design.md

        === Threading model (load-bearing, read before editing) ===

        The serial thread must NEVER wait on the app thread: at
        420 kbaud the kernel tty buffer holds ~1 s of traffic at
        100 Hz framing, and a stalled reader loses bytes with no
        recovery. Three rules enforce that property:

        1. The serial thread exclusively owns the port (m_fd), the
           parser, and its working state (m_work). No lock guards
           them; nothing else touches them after OnStartUp().

        2. Cross-thread traffic is two small structs:
             m_snap  (serial -> app): RcSnapshot, published with
                     try_lock - if the app holds the lock, the
                     serial thread SKIPS and republishes next pass
                     (<= 20 ms). It never blocks.
             m_telem_in (app -> serial): telemetry inputs from
                     OnNewMail, snapshotted with try_lock - if
                     contended, telemetry is deferred one pass.

        3. The app thread copies m_snap under the lock, releases,
           then does ALL Notify()/PostReport()/formatting from the
           copy. INVARIANT: no mutex is ever held across a MOOS
           call. buildReport() takes no lock at all - the recursive
           lock deadlock (Iterate -> PostReport -> buildReport, all
           on m_mutex) that shipped in the first version of this
           app is structurally impossible, not just avoided.

        The app thread also calls refresh() on its COPY of the
        link state before publishing, so a dead or wedged serial
        thread cannot freeze RC_CONNECTED=true and silently defeat
        the deadman in iBBNavigatorInterface.
*************************************************************/

#ifndef RCInterface_HEADER
#define RCInterface_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <cstdarg>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <ctime>

#include "crsf_parser.h"
#include "command_envelope.h"
#include "crsf_frames.h"
#include "rc_link_state.h"

// Channel scaling constants. Deliberately the SBUS values (max
// 1812, not CRSF's 1811) so ScaleJoystick produces bit-identical
// axis output to iRCReader. A real CRSF 1811 is inside the clamp.
#define RC_TICKS_MIN  172
#define RC_TICKS_MID  992
#define RC_TICKS_MAX  1812

#define RC_NUM_CHANNELS 16

// Guard-band decode thresholds for discrete channels (RC
// Contract v2): state 1 below, top state above; for a 2-pos
// channel the middle is a DEAD BAND, not a state. An unmixed or
// faulted channel parks at 992 — inside the dead band by design,
// so a missing mix reads as a config fault instead of landing on
// the armed edge of a 2-pos switch (which is exactly what naive
// midpoint decoding would do).
#define RC_GUARD_LOW   600
#define RC_GUARD_HIGH  1400

// Vehicle-side floor for CH11 THRUST_LIMIT percent, applied here
// and only here so "why won't it move" cannot happen.
#define RC_THRUST_LIMIT_FLOOR_PCT 25.0

//---------------------------------------------------------
// RcSnapshot: everything the app thread needs from the serial
// thread, copied whole. Kept small (a few hundred bytes) so the
// critical section is a memcpy and nothing else.

struct RcSnapshot {
  uint16_t    channels[RC_NUM_CHANNELS] = {0};
  RcLinkState link_state;

  CrsfLinkStats link = {};
  bool          have_link    = false;
  uint64_t      last_link_us = 0;

  bool     port_open       = false;
  uint64_t crc_fails       = 0;
  uint64_t bytes_discarded = 0;

  uint64_t sent_frames     = 0;
  uint64_t dropped_writes  = 0;   // clean drop: nothing hit the wire
  uint64_t partial_writes  = 0;   // torn frame DID hit the wire
};

//---------------------------------------------------------
// RcTelemInputs: telemetry sources harvested from MOOS mail,
// handed to the serial thread for encoding. The *_us stamps gate
// forwarding: inputs older than the staleness window are skipped
// so handset sensors go stale honestly.

struct RcTelemInputs {
  double   volts = 0, amps = 0;
  bool     have_volts = false, have_amps = false;
  uint64_t volts_us = 0;

  double   nav_lat = 0, nav_lon = 0;
  bool     have_pos = false;
  uint64_t pos_us = 0;

  double   nav_spd = 0, nav_hdg = 0;

  double   nav_x = 0, nav_y = 0;
  bool     have_xy = false;
  uint64_t xy_us = 0;

  bool rc_mode = false;
  bool deadman = false;
};

// Uplink LQ below this reports link=DEGRADED while frames are
// still valid. 80 comes from the measured 1.1 s linear decay
// 100 -> 0 on TX loss (docs/archive/rc/rc_characterization.md):
// it buys roughly a second of warning without firing on the
// ordinary 90s seen at range.
static const int RC_LINK_LQ_DEGRADED = 80;

class RCInterface : public AppCastingMOOSApp
{
 public:
   RCInterface();
   ~RCInterface();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);

 private: // Serial thread body (see threading model above)
   void   SerialThreadFunction();
   bool   openPort();
   void   closePort();
   void   handleFrame(const CrsfFrame &frame, uint64_t now_us);
   void   sendTelemetry(uint64_t now_us);
   bool   writeFrameNonBlocking(const std::vector<uint8_t> &f);
   void   publishSnapshot();
   static uint64_t microsNow();

 private: // Scaling helpers
   // Axis scaling: identical math to iRCReader.
   double ScaleJoystick(uint16_t value);
   // Discrete decode per Contract v2 guard bands. Returns the
   // state (1..num_states), or 0 for a 2-pos dead-band fault.
   int    MapSwitchGuarded(uint16_t value, int num_states);

 private: // Configuration (written in OnStartUp, read-only after)
   std::string m_device;
   bool        m_telemetry_enabled;
   double      m_telem_hz;   // battery + flight-mode rate
   double      m_gps_hz;     // GPS + distance rate

   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

 private: // Serial-thread-private (NO lock: single owner)
   int         m_fd;
   CrsfParser  m_parser;
   RcSnapshot  m_work;          // working copy, published to m_snap
   uint64_t    m_next_telem_us;
   uint64_t    m_next_gps_us;

   std::thread       m_serial_thread;
   std::atomic<bool> m_running;

 private: // Cross-thread handoff (the ONLY shared state)
   std::mutex  m_snap_mutex;    // guards m_snap only
   RcSnapshot  m_snap;

   std::mutex     m_telem_mutex;  // guards m_telem_in only
   RcTelemInputs  m_telem_in;

 private: // App-thread-private (no lock needed)
   RcSnapshot m_report;         // last snapshot copied in Iterate
   double     m_scaled_channels[RC_NUM_CHANNELS];
   bool       m_protocol_posted;   // RC_PROTOCOL published once
   bool       m_prev_port_open;    // transition -> appcast event
   bool       m_prev_failsafe;     // transition -> appcast event

   // CH5 KILL dead-band latch: a config fault must never silently
   // kill an autonomous boat, nor un-kill a killed one
   // (rc_model_build_yip.md), so an invalid KILL value holds the
   // last valid state instead of defaulting.
   int        m_kill_state;

   // Latched guarded-switch state for the COMMAND CONTRACT.
   //
   // These differ from the RC_CH* scalars on purpose. On link
   // loss the scalars fall to their operator-absent defaults
   // (CH6 -> 1 = AUTO), which is right for that contract. The
   // command contract must NOT do that: publishing
   // mode=NON_MANUAL because the link died would tell the arbiter
   // the operator chose autonomy, and the boat would hand itself
   // to the back seat at the exact moment the operator lost it.
   // Design doc 5.1 forbids it; invariant 6 is what it protects.
   //
   // So these RETAIN their last valid decode across a dropout,
   // and RC_INPUT_STATE carries valid=0 instead. The arbiter then
   // sees "the operator still holds MANUAL and cannot command" and
   // hard-stops with RC_INVALID -- fail-closed, no handover.
   //
   // 0 = never decoded (before the first valid frame).
   int        m_mode_state;
   int        m_deadman_state;
   double     m_authority_pct;

   // RC_INPUT_STATE contract identity and config.
   std::string m_rc_input_var;
   std::string m_rc_input_epoch;
   uint64_t    m_rc_input_seq;
   bool        m_thrust_limit_enable;

   void publishRcInputState(bool frame_valid, bool rc_connected, bool failsafe);

   // CH12 MARK edge detector: a rising edge (1->2, SH pressed)
   // publishes an incrementing RC_MARK count - one countable event
   // per press for the log, while the level keeps going out as
   // RC_CH12. State is not touched during disconnects, so a press
   // held across a dropout cannot double-count.
   int          m_mark_last_state;
   unsigned int m_mark_count;
};

#endif
