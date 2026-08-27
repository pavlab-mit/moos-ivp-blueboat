/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iRCInterface/RCInterface.cpp
   Last Ed: 2026-08-12
     Brief:
        CRSF RC interface. One app owns the RC serial port in
        both directions: RC channels + link stats in, telemetry
        (battery / GPS / flight mode / distance) out.

        The MOOS-facing RC_* contract follows RC Contract v2
        (docs/rc_handset_architecture.md, frozen 2026-08-11):
        axes CH1-4 keep iRCReader's exact scaling; discrete
        channels follow the TX16S semantic map with guard-band
        decoding; CH11 is a floored thrust-limit percent.

        Threading model documented in RCInterface.h. The short
        form: serial thread owns the port and never waits on a
        lock; the app thread reads a snapshot and never holds a
        lock across a MOOS call.
*************************************************************/

#include <iterator>
#include <cstring>
#include <cmath>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>

#include "MBUtils.h"
#include "ACTable.h"
#include "RCInterface.h"

using namespace std;

// Linux-only 420k baud plumbing (termios2 BOTHER), the same
// scheme rc_probe and rc_telem_test use. Standard termios has no
// B420000 constant.
#define RC_BOTHER  0010000
#define RC_TCGETS2 0x802C542A
#define RC_TCSETS2 0x402C542B
#define CRSF_BAUDRATE 420000

// Telemetry inputs older than this are not forwarded, so the
// handset sensors go stale instead of displaying frozen values.
#define TELEM_INPUT_STALE_US  5000000ull

//---------------------------------------------------------
// Constructor()

RCInterface::RCInterface()
{
  m_device            = "/dev/ttyAMA1";
  m_telemetry_enabled = true;
  m_telem_hz          = 2.0;
  m_gps_hz            = 1.0;

  m_debug = false;
  m_debug_stream = nullptr;

  m_fd = -1;
  m_running = true;
  m_next_telem_us = 0;
  m_next_gps_us   = 0;

  // m_work / m_snap / m_report / m_telem_in zero themselves via
  // member initializers.

  for (int i = 0; i < RC_NUM_CHANNELS; i++)
    m_scaled_channels[i] = 0;

  m_protocol_posted = false;
  m_prev_port_open  = false;
  m_prev_failsafe   = false;

  // Until a valid KILL decode arrives, the latch holds the
  // operator-absent default: RUNNING.
  m_kill_state = 1;

  // 0 = never decoded. The contract publishes mode=UNKNOWN until
  // a real frame arrives, which the arbiter reads as not-manual
  // so a boat launched with the handset off can still run
  // (plan decision (e)).
  m_mode_state    = 0;
  m_deadman_state = 0;
  m_authority_pct = 100.0;

  m_rc_input_var        = "RC_INPUT_STATE";
  m_rc_input_seq        = 0;
  m_thrust_limit_enable = false;

  m_mark_last_state = 1;
  m_mark_count = 0;
}

//---------------------------------------------------------
// Destructor

RCInterface::~RCInterface()
{
  m_running = false;
  if (m_serial_thread.joinable()) {
    m_serial_thread.join();
    dbg_print("Serial thread joined on shutdown\n");
  }
  closePort();
}

//---------------------------------------------------------
// Procedure: microsNow()

uint64_t RCInterface::microsNow()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

//---------------------------------------------------------
// Procedure: OnNewMail()
//
// Ingests the telemetry sources into m_telem_in. This is the
// app -> serial handoff; the lock is scoped per message and is
// never held across a MOOS call. RC decode never depends on
// mail; a dead MOOSDB cannot take down RC input.

bool RCInterface::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  const uint64_t now = microsNow();

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    std::lock_guard<std::mutex> lock(m_telem_mutex);

    if (key == "NVGR_ROLLING_VOLTAGE" && msg.IsDouble()) {
      m_telem_in.volts = msg.GetDouble();
      m_telem_in.have_volts = true;
      m_telem_in.volts_us = now;
    }
    else if (key == "NVGR_ROLLING_CURRENT" && msg.IsDouble()) {
      m_telem_in.amps = msg.GetDouble();
      m_telem_in.have_amps = true;
    }
    else if (key == "NAV_LAT" && msg.IsDouble()) {
      m_telem_in.nav_lat = msg.GetDouble();
      m_telem_in.pos_us = now;
    }
    else if (key == "NAV_LONG" && msg.IsDouble()) {
      m_telem_in.nav_lon = msg.GetDouble();
      m_telem_in.have_pos = true;
    }
    else if (key == "NAV_SPEED" && msg.IsDouble())
      m_telem_in.nav_spd = msg.GetDouble();
    else if (key == "NAV_HEADING" && msg.IsDouble())
      m_telem_in.nav_hdg = msg.GetDouble();
    else if (key == "NAV_X" && msg.IsDouble()) {
      m_telem_in.nav_x = msg.GetDouble();
      m_telem_in.xy_us = now;
    }
    else if (key == "NAV_Y" && msg.IsDouble()) {
      m_telem_in.nav_y = msg.GetDouble();
      m_telem_in.have_xy = true;
    }
    else if (key == "BB_CMD_AUTHORITY" && msg.IsString())
      m_telem_in.authority = msg.GetString();
    else if (key == "NVGR_STOP_REASON" && msg.IsString())
      m_telem_in.deadman = (msg.GetString() == "RC_DEADMAN");
    else if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: dbg_print()
//
// APP THREAD ONLY. It does a filesystem round-trip per call and
// touches AppCasting state on failure; calling it from the serial
// thread would put an SD-card write on the decode path (the
// original version did exactly that, under the lock no less).

bool RCInterface::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      va_end(args);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      va_end(args);
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RCInterface::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: openPort()
//
// 420000 baud 8N1 via termios2 BOTHER. No parity, so any Pi
// UART frames CRSF — the mini-UART/disable-bt constraint that
// governed SBUS does not apply. OPOST must be off on the TX
// path: with ONLCR active the driver rewrites 0x0A to 0x0D 0x0A
// and the battery frame's length byte is exactly 0x0A, which
// silently corrupts every battery frame while others pass.

bool RCInterface::openPort()
{
  struct stat st;
  if (stat(m_device.c_str(), &st) != 0)
    return false;

  int fd = open(m_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return false;

#ifdef TIOCEXCL
  // Exclusive mode: any second open() of this tty fails with
  // EBUSY (except root). Two readers on one tty split the byte
  // stream between them — the suspected cause of the anomaly in
  // rc_characterization.md — so make it impossible, not just
  // detectable.
  ioctl(fd, TIOCEXCL);
#endif

  struct termios tio;
  if (tcgetattr(fd, &tio) < 0) { close(fd); return false; }
  cfsetispeed(&tio, B38400);
  cfsetospeed(&tio, B38400);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;    // 8N1
  tio.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(INLCR | ICRNL | INPCK | ISTRIP |
                   IGNBRK | BRKINT | PARMRK);
  tio.c_oflag &= ~OPOST;
#ifdef ONLCR
  tio.c_oflag &= ~ONLCR;
#endif
#ifdef OCRNL
  tio.c_oflag &= ~OCRNL;
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return false; }

#ifdef __linux__
  struct custom_termios2 {
    tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed, c_ospeed;
  };
  struct custom_termios2 tio2;
  if (ioctl(fd, RC_TCGETS2, &tio2) < 0) { close(fd); return false; }
  tio2.c_cflag &= ~0000017;
  tio2.c_cflag |= RC_BOTHER;
  tio2.c_ispeed = CRSF_BAUDRATE;
  tio2.c_ospeed = CRSF_BAUDRATE;
  if (ioctl(fd, RC_TCSETS2, &tio2) < 0) { close(fd); return false; }
#endif

  tcflush(fd, TCIFLUSH);
  m_fd = fd;
  return true;
}

//---------------------------------------------------------
// Procedure: closePort()

void RCInterface::closePort()
{
  if (m_fd >= 0) {
    close(m_fd);
    m_fd = -1;
  }
}

//---------------------------------------------------------
// Procedure: writeFrameNonBlocking()
//
// Telemetry must never stall the RC decode path — RC control
// latency is the safety-critical property; telemetry is not. A
// frame that cannot be written immediately is DROPPED, not
// queued and not retried: the next one carries fresher data.
//
// A PARTIAL write is counted separately from a clean drop: the
// accepted bytes already went on the wire as a torn frame (the
// receiver CRC-rejects it), so the appcast can distinguish
// "telemetry backpressure" from "we put garbage on the TX path".

bool RCInterface::writeFrameNonBlocking(const std::vector<uint8_t> &f)
{
  ssize_t n = write(m_fd, f.data(), f.size());
  if (n == (ssize_t)f.size()) {
    m_work.sent_frames++;
    return true;
  }
  if (n > 0)
    m_work.partial_writes++;
  else
    m_work.dropped_writes++;
  return false;
}

//---------------------------------------------------------
// Procedure: publishSnapshot()
//
// Serial -> app handoff. try_lock, never lock: if the app thread
// is mid-copy, skip — the next loop pass (<= 20 ms) republishes.
// The serial thread's worst-case wait on this lock is zero, by
// construction. Contention is ~1e-5 (16 Hz x ~1 us hold time),
// so a skipped publish is rare and costs one pass of snapshot
// staleness, invisible under a 62.5 ms AppTick.

void RCInterface::publishSnapshot()
{
  std::unique_lock<std::mutex> lk(m_snap_mutex, std::try_to_lock);
  if (lk.owns_lock())
    m_snap = m_work;
}

//---------------------------------------------------------
// Procedure: handleFrame()
//
// Serial thread, serial-private state, no lock. CRC already
// establishes integrity, so unlike SBUS there is no range-check
// validation layer and no validated_channels knob.

void RCInterface::handleFrame(const CrsfFrame &frame, uint64_t now_us)
{
  if (frame.type == CRSF_FRAMETYPE_RC_CHANNELS) {
    CrsfChannels ch;
    if (!crsfDecodeChannels(frame.payload, ch))
      return;

    for (int i = 0; i < RC_NUM_CHANNELS; i++)
      m_work.channels[i] = ch.channel[i];
    m_work.link_state.onChannelsFrame(now_us);
  }
  else if (frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
    CrsfLinkStats ls;
    if (!crsfDecodeLinkStats(frame.payload, ls))
      return;

    m_work.link = ls;
    m_work.have_link = true;
    m_work.last_link_us = now_us;
    m_work.link_state.onLinkStats(ls.uplink_lq, now_us);
  }
  // Other frame types (0x1C heartbeat etc.): ignore silently.
}

//---------------------------------------------------------
// Procedure: sendTelemetry()
//
// Paced from the serial thread. Inputs are snapshotted with
// try_lock — if OnNewMail holds the lock, telemetry is deferred
// one pass (pacing deadlines are NOT advanced, so nothing is
// skipped, only delayed <= 20 ms). Stale inputs are skipped, not
// repeated — the handset sensor going stale/red is the honest
// signal.

void RCInterface::sendTelemetry(uint64_t now_us)
{
  if (!m_telemetry_enabled || m_fd < 0)
    return;

  bool do_telem = (now_us >= m_next_telem_us);
  bool do_gps   = (m_gps_hz > 0) && (now_us >= m_next_gps_us);
  if (!do_telem && !do_gps)
    return;

  RcTelemInputs in;
  {
    std::unique_lock<std::mutex> lk(m_telem_mutex, std::try_to_lock);
    if (!lk.owns_lock())
      return;   // contended: retry next pass, deadlines unchanged
    in = m_telem_in;
  }

  bool send_batt = in.have_volts && (now_us - in.volts_us < TELEM_INPUT_STALE_US);
  bool send_gps  = in.have_pos   && (now_us - in.pos_us   < TELEM_INPUT_STALE_US);
  bool send_dist = in.have_xy    && (now_us - in.xy_us    < TELEM_INPUT_STALE_US);

  std::vector<uint8_t> f;

  if (do_telem) {
    m_next_telem_us = now_us + (uint64_t)(1e6 / m_telem_hz);

    if (send_batt) {
      // Zero the fields we have no source for (capacity,
      // remaining) rather than inventing them.
      crsfEncodeBattery(in.volts, in.have_amps ? in.amps : 0.0, 0, 0, f);
      writeFrameNonBlocking(f);
    }

    // Vehicle-side liveness: EdgeTX shows this string natively as
    // the FM sensor. Confirmed-mode loop: the handset banner shows
    // the COMMANDED mode, FM shows what the vehicle is ACTUALLY
    // in -- now sourced from the arbiter's authority decision
    // rather than the retired Navigator RC-mode flag.
    const char *mode = "AUTO";
    if      (in.deadman)                 mode = "DEADMAN";
    else if (in.authority == "RC")       mode = "RC";
    else if (in.authority == "TELEOP")   mode = "TELEOP";
    else if (in.authority == "NONE")     mode = "STOP";
    else if (in.authority == "AUTONOMY") mode = "AUTO";
    crsfEncodeFlightMode(mode, f);
    writeFrameNonBlocking(f);
  }

  if (do_gps) {
    m_next_gps_us = now_us + (uint64_t)(1e6 / m_gps_hz);

    if (send_gps) {
      crsfEncodeGps(in.nav_lat, in.nav_lon, in.nav_spd, in.nav_hdg, 0.0, 0, f);
      writeFrameNonBlocking(f);
    }
    if (send_dist) {
      // Distance from the MOOS origin; decoded to the "Dst"
      // sensor by dstrx.lua on the handset.
      crsfEncodeCustomDistance(hypot(in.nav_x, in.nav_y), f);
      writeFrameNonBlocking(f);
    }
  }
}

//---------------------------------------------------------
// Procedure: SerialThreadFunction()
//
// Owns the port and all of m_work. Reads/decodes continuously;
// telemetry writes are paced and non-blocking; the snapshot
// publish is try_lock. Nothing in this loop can wait on the app
// thread. If the port cannot be opened it retries every 2 s
// rather than giving up — the receiver may power up after MOOS
// does.

void RCInterface::SerialThreadFunction()
{
  uint64_t next_open_attempt = 0;
  uint8_t  rbuf[512];
  std::vector<CrsfFrame> frames;

  while (m_running) {
    const uint64_t now = microsNow();

    if (m_fd < 0) {
      if (now >= next_open_attempt) {
        next_open_attempt = now + 2000000ull;
        openPort();
      }
      // Keep the staleness clock and port status honest while
      // the port is down.
      m_work.port_open = (m_fd >= 0);
      m_work.link_state.refresh(microsNow());
      publishSnapshot();
      if (m_fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }

    // Wait up to 20 ms for bytes; also bounds the telemetry,
    // staleness-update, and snapshot-republish latency.
    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(m_fd, &rf);
    struct timeval tv = {0, 20000};
    int rv = select(m_fd + 1, &rf, nullptr, nullptr, &tv);

    if (rv > 0) {
      ssize_t n = read(m_fd, rbuf, sizeof(rbuf));
      if (n > 0) {
        frames.clear();
        m_parser.feed(rbuf, (size_t)n, frames);
        for (const auto &fr : frames)
          handleFrame(fr, microsNow());
      } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
        // Port died (unplugged USB adapter etc.) — reopen path.
        closePort();
        continue;
      }
    }

    const uint64_t after = microsNow();
    m_work.link_state.refresh(after);
    m_work.port_open       = true;
    m_work.crc_fails       = m_parser.framesCrcFail();
    m_work.bytes_discarded = m_parser.bytesDiscarded();
    publishSnapshot();

    sendTelemetry(after);
  }
}

//---------------------------------------------------------
// Scale joystick value to -100 to 100. Identical math and
// constants to iRCReader::ScaleJoystick.

double RCInterface::ScaleJoystick(uint16_t value)
{
  if (value < RC_TICKS_MIN) value = RC_TICKS_MIN;
  if (value > RC_TICKS_MAX) value = RC_TICKS_MAX;

  double center = (RC_TICKS_MAX + RC_TICKS_MIN) / 2.0;
  double range  = (RC_TICKS_MAX - RC_TICKS_MIN) / 2.0;

  return ((value - center) / range) * 100.0;
}

//---------------------------------------------------------
// Map a discrete channel to a state with Contract v2 guard
// bands. For 2-pos channels the centre is a dead band and
// returns 0 (invalid): an unmixed channel emits 992, and naive
// midpoint decoding would land that on a real state — for
// KILL/MODE, the armed edge. For 3-pos channels the centre band
// IS the middle state (the wire value there is a legitimate
// 992), so no dead band exists.

int RCInterface::MapSwitchGuarded(uint16_t value, int num_states)
{
  if (value < RC_GUARD_LOW)
    return 1;
  if (value > RC_GUARD_HIGH)
    return num_states;
  if (num_states == 3)
    return 2;
  return 0;   // 2-pos dead band: config fault, caller decides
}

//---------------------------------------------------------
// Procedure: Iterate()
//
// Copy the snapshot, release the lock, THEN publish. The publish
// block below is the load-bearing contract. Order matters:
// RC_CONNECTED is published BEFORE the safe-default RC_CH* values
// within an iterate, so consumers that gate RC_CH* mail on
// RC_CONNECTED (iBBNavigatorInterface's deadman and mode latch)
// see the gate close before the tokens arrive.
//
// INVARIANT: no mutex is held anywhere in this function beyond
// the snapshot copy — in particular, none across Notify() or
// PostReport().

bool RCInterface::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (!m_protocol_posted) {
    // Once, for log forensics: which decoder produced this log.
    Notify("RC_PROTOCOL", "crsf");
    m_protocol_posted = true;
  }

  {
    std::lock_guard<std::mutex> lock(m_snap_mutex);
    m_report = m_snap;
  }

  // Reader-side staleness re-evaluation on OUR copy with OUR
  // clock. If the serial thread is dead or wedged, its last
  // snapshot froze with rc_connected=true — this refresh flips it
  // false once the frames age out, so a wedged reader thread
  // cannot defeat the RC deadman in iBBNavigatorInterface.
  const uint64_t now = microsNow();
  m_report.link_state.refresh(now);

  const bool frame_valid  = m_report.link_state.frameValid();
  const bool rc_connected = m_report.link_state.rcConnected();
  const bool failsafe     = m_report.link_state.failsafe();

  // Serial-side transitions surfaced as appcast events (the
  // serial thread itself must not touch AppCasting state).
  if (m_report.port_open != m_prev_port_open) {
    reportEvent(m_report.port_open ? ("Opened " + m_device)
                                   : ("Lost " + m_device + ", retrying"));
    m_prev_port_open = m_report.port_open;
  }
  if (failsafe != m_prev_failsafe) {
    reportEvent(failsafe ? "FAILSAFE: uplink LQ 0, receiver declares dead uplink"
                         : "Failsafe cleared");
    m_prev_failsafe = failsafe;
  }

  Notify("RC_FRAME_VALID", frame_valid  ? "true" : "false");
  Notify("RC_CONNECTED",   rc_connected ? "true" : "false");

  if (rc_connected)
  {
    // === Contract v2 channel map (rc_handset_architecture.md,
    // === frozen 2026-08-11). State 1 on every discrete channel
    // === is the operator-absent default by design.

    // Block A — axes CH1-4, scaled to -100..100 (identical math
    // to iRCReader; the sign/polarity contract lives in the
    // handset mixes, never here).
    for (int i = 0; i < 4; i++) {
      m_scaled_channels[i] = ScaleJoystick(m_report.channels[i]);
      Notify("RC_CH" + intToString(i+1), m_scaled_channels[i]);
    }

    // CH5 KILL (SW1): 1=RUNNING 2=KILLED. A dead-band fault
    // LATCHES the last valid state — a config fault must never
    // silently kill an autonomous boat, nor un-kill a killed one.
    {
      int s = MapSwitchGuarded(m_report.channels[4], 2);
      if (s == 0) {
        reportRunWarning("RC_CH5 (KILL) in guard dead band; "
                         "latching last valid state");
        s = m_kill_state;
      } else
        m_kill_state = s;
      m_scaled_channels[4] = s;
      Notify("RC_CH5", m_scaled_channels[4]);
    }

    // Remaining discrete channels. 2-pos dead band -> safe
    // state 1 (the operator-absent default) + run warning; the
    // 3-pos channels (ACTION, PAYLOAD_MODE) have no dead band.
    //   CH6  MODE        (SF)  1=AUTO    2=RC
    //   CH7  ACTION      (SB)  1=HOLD    2=RESUME  3=RETURN
    //   CH8  DEADMAN_EN  (SW2) 1=ENABLED 2=DISABLED
    //   CH9  PAYLOAD_PWR (SW3) 1=OFF     2=ON
    //   CH10 PAYLOAD_MODE(SC)  3-pos
    //   CH12 MARK        (SH)  momentary, 2=pressed
    static const struct { int idx; int states; const char *name; }
      discretes[] = {
        {5,  2, "MODE"},        {6,  3, "ACTION"},
        {7,  2, "DEADMAN_EN"},  {8,  2, "PAYLOAD_PWR"},
        {9,  3, "PAYLOAD_MODE"},{11, 2, "MARK"},
      };
    for (const auto &d : discretes) {
      int s = MapSwitchGuarded(m_report.channels[d.idx], d.states);
      if (s == 0) {
        reportRunWarning("RC_CH" + intToString(d.idx+1) + " (" +
                         d.name + ") in guard dead band; safe state 1");
        s = 1;
      }
      m_scaled_channels[d.idx] = s;
      Notify("RC_CH" + intToString(d.idx+1), (double)s);

      // Latch the two the command contract carries. Only while
      // connected: a dropout must never rewrite the operator's
      // last real choice (see RCInterface.h).
      if (d.idx == 5) m_mode_state    = s;   // CH6 MODE
      if (d.idx == 7) m_deadman_state = s;   // CH8 DEADMAN_EN
    }

    // CH12 MARK (SH momentary): rising edge -> one countable event.
    // The level is already published as RC_CH12 above; RC_MARK is
    // what consumers (pLogger, operators reading the alog) actually
    // want - one increment per press, no re-fires while held.
    {
      int s = (int)m_scaled_channels[11];
      if (s == 2 && m_mark_last_state == 1) {
        m_mark_count++;
        Notify("RC_MARK", (double)m_mark_count);
        reportEvent("MARK #" + intToString((int)m_mark_count) +
                    " from handset");
      }
      m_mark_last_state = s;
    }

    // CH11 THRUST_LIMIT (S1 pot): full range on the wire,
    // published as percent with the floor applied here and only
    // here (rc_model_build_yip.md keeps the handset side linear).
    {
      uint16_t v = m_report.channels[10];
      if (v < RC_TICKS_MIN) v = RC_TICKS_MIN;
      if (v > RC_TICKS_MAX) v = RC_TICKS_MAX;
      double pct = (double)(v - RC_TICKS_MIN) /
                   (double)(RC_TICKS_MAX - RC_TICKS_MIN) * 100.0;
      if (pct < RC_THRUST_LIMIT_FLOOR_PCT)
        pct = RC_THRUST_LIMIT_FLOOR_PCT;
      m_scaled_channels[10] = pct;
      m_authority_pct = pct;
      Notify("RC_CH11", pct);
    }

    // CH13-16: reserved (unmixed, 992 on the wire); raw values.
    for (int i = 12; i < RC_NUM_CHANNELS; i++) {
      m_scaled_channels[i] = m_report.channels[i];
      Notify("RC_CH" + intToString(i+1), m_report.channels[i]);
    }

    // CRSF carries no CH17/18 digital flags; published as false
    // for contract parity with the SBUS-era logs.
    Notify("RC_CH17", "false");
    Notify("RC_CH18", "false");
    Notify("RC_FRAME_LOST", frame_valid ? "false" : "true");
    Notify("RC_FAILSAFE", failsafe ? "true" : "false");
  }
  else
  {
    // === Disconnected fallback: publish safe defaults for ALL
    // === channels, per Contract v2's rule that state 1 on every
    // === discrete channel is the state the vehicle should assume
    // === when the operator vanishes.
    //
    // Axes     CH1-4       -> 0   (zero thrust commanded)
    // Discrete CH5-10,CH12 -> 1   (RUNNING, AUTO, HOLD, deadman
    //                              ENABLED, payload OFF, no mark)
    // Analog   CH11        -> 100 (no operator thrust limit — a
    //                              lingering 25% cap must not
    //                              cripple autonomous ops)
    // Reserved CH13-16     -> mid (no command bias)
    //
    // These are "operator unavailable" tokens, not commanded
    // positions. The RC_DEADMAN watchdog in iBBNavigatorInterface
    // refreshes its freshness timestamp ONLY on RC_CONNECTED=true,
    // so these publishes do NOT defeat the watchdog.
    Notify("RC_CH1", 0.0);
    Notify("RC_CH2", 0.0);
    Notify("RC_CH3", 0.0);
    Notify("RC_CH4", 0.0);
    Notify("RC_CH5", 1.0);
    Notify("RC_CH6", 1.0);
    Notify("RC_CH7", 1.0);
    Notify("RC_CH8", 1.0);
    Notify("RC_CH9", 1.0);
    Notify("RC_CH10", 1.0);
    Notify("RC_CH11", 100.0);
    Notify("RC_CH12", 1.0);
    for (int i = 12; i < RC_NUM_CHANNELS; i++) {
      Notify("RC_CH" + intToString(i+1), (double)RC_TICKS_MID);
    }
  }

  // The command contract. Published EVERY iterate, connected or
  // not, so the arbiter's lease sees a live producer and can tell
  // "operator present but cannot command" from "producer dead".
  publishRcInputState(frame_valid, rc_connected, failsafe);

  // New, additive: direct link health from CRSF 0x14. Uplink
  // only — ELRS never populates the downlink fields (see
  // crsf_frames.h). Published while stats are fresh so the last
  // value does not masquerade as live after link loss.
  if (m_report.have_link &&
      (now - m_report.last_link_us) < 2000000ull) {
    Notify("RC_LINK_LQ",   (double)m_report.link.uplink_lq);
    Notify("RC_LINK_RSSI", (double)m_report.link.uplink_rssi_ant1);
    Notify("RC_LINK_SNR",  (double)m_report.link.uplink_snr);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: publishRcInputState()
//
// RC_INPUT_STATE: conditioned semantic operator intent, as one
// coherent snapshot (design doc 5.1).
//
// CH3 and CH1 are ALREADY plus-is-ahead and plus-is-starboard in
// the handset mixes, which is the contract's convention exactly.
// So surge and yaw are those channels verbatim -- no sign
// transform lives here, and none should. This is what retires
// rc_stick_convention: the v1/v2 knob existed because the
// Navigator's RC mixer implemented the legacy RadioLink wire, and
// once RC emits semantic values there is nothing left to get
// backwards. The old "turns correct, throttle reversed" failure
// mode stops being expressible.
//
// This function owns NO mixing, NO ESC signs, and NO authority
// policy. It reports what the operator is asking for and how much
// of that can be trusted; the arbiter decides whether it counts.

void RCInterface::publishRcInputState(bool frame_valid, bool rc_connected,
                                      bool failsafe)
{
  bb::SemanticCommand cmd;
  cmd.version     = bb::kCommandContractVersion;
  cmd.producer    = "iRCInterface";
  cmd.epoch       = m_rc_input_epoch;
  cmd.seq         = ++m_rc_input_seq;
  cmd.source_time = MOOSTime();

  // valid means "these sticks are trustworthy this instant".
  // A single bad frame drops it, which is sharper than the
  // debounced rc_connected and is what gates thrust.
  cmd.valid = rc_connected && frame_valid && !failsafe;

  // Sticks pass through only while the frame is good. On a bad
  // frame we publish zero AND valid=0: the zero keeps a stale
  // deflection off the wire, the flag is what the arbiter acts on.
  if (cmd.valid) {
    cmd.surge = m_scaled_channels[2];   // CH3 THRUST,  + = ahead
    cmd.yaw   = m_scaled_channels[0];   // CH1 STEER,   + = starboard
  } else {
    cmd.surge = 0.0;
    cmd.yaw   = 0.0;
  }

  // MODE and KILL are RETAINED across a dropout -- the whole
  // point. See the note in RCInterface.h.
  const char* mode = "UNKNOWN";
  if (m_mode_state == 2)      mode = "MANUAL";
  else if (m_mode_state == 1) mode = "NON_MANUAL";
  cmd.extra["mode"] = mode;

  cmd.extra["kill"] = (m_kill_state == 2) ? "1" : "0";

  // link state, using the MEASURED failsafe signature from
  // docs/archive/rc/rc_characterization.md (2026-08-11):
  //
  //   - 0x16 RC frames stop DEAD on TX loss. No fade, no
  //     fabricated values, no failsafe frames. So frame staleness
  //     is the complete and correct loss detector, and there is
  //     nothing to misread on the way down.
  //   - 0x14 link stats keep coming for ~1.1 s with uplink LQ
  //     decaying linearly 100 -> 0, then pin at 0 until the
  //     receiver goes silent at ~3.5 s.
  //
  // That decay is roughly a second of genuine early warning, and
  // it is the only thing that distinguishes "about to lose the
  // operator" from "fine". DEGRADED reports it while the frames
  // are still valid, so the log shows the link dying before the
  // command actually stops -- an incident review can then tell a
  // range problem from an app fault.
  const char* link = "LOST";
  const bool lq_fresh = m_report.have_link &&
                        (microsNow() - m_report.last_link_us) < 2000000ull;
  const bool lq_weak  = lq_fresh && (m_report.link.uplink_lq < RC_LINK_LQ_DEGRADED);

  if (rc_connected && frame_valid && !failsafe)
    link = lq_weak ? "DEGRADED" : "VALID";
  else if (rc_connected)
    link = "DEGRADED";
  cmd.extra["link"] = link;
  if (lq_fresh) cmd.extra["lq"] = intToString(m_report.link.uplink_lq);

  cmd.extra["deadman_switch"] =
      (m_deadman_state == 2) ? "DISABLED" : "ENABLED";

  // CH11 THRUST_LIMIT: the operator authority cap, still behind
  // its enable flag. Disabled publishes full authority rather
  // than omitting the field, so a log always shows what cap was
  // in force. The arbiter applies it to MANUAL sources only.
  cmd.authority_limit = m_thrust_limit_enable ? m_authority_pct : 100.0;

  Notify(m_rc_input_var, bb::serialize_command(cmd));

  // --- HARDWARE SIDEBANDS ---------------------------------
  //
  // These two exist for iBBNavigatorInterface and are deliberately
  // PLAIN BOOLEANS, not fields inside RC_INPUT_STATE.
  //
  // Both bypass the command path entirely: the kill must stop the
  // boat when the arbiter or the mixer has failed, and the deadman
  // exists precisely for running code that is not trusted yet. A
  // sideband that requires parsing the same contract as the
  // command path can fail the same way the command path fails,
  // which defeats the point of having one.
  //
  // RC_KILL_ASSERTED is latched state and survives link loss --
  // a killed boat stays killed when the handset goes away.
  // RC_LINK_ALIVE is instantaneous link presence, which is what
  // the opt-in deadman watches.
  Notify("RC_KILL_ASSERTED", (m_kill_state == 2) ? "true" : "false");
  Notify("RC_LINK_ALIVE",    rc_connected ? "true" : "false");
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool RCInterface::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "rc_input_var") {
      m_rc_input_var = value;
      handled = true;
    }
    else if (param == "thrust_limit_enable") {
      // CH11 operator authority cap. Off by default so a boat
      // that has never been told about the pot cannot be
      // silently limited by one; the fleet plug turns it on.
      handled = setBooleanOnString(m_thrust_limit_enable, value);
    }
    else if (param == "device")
    {
      // UART wired to the RC receiver, e.g. /dev/ttyAMA1
      // (Navigator SERIAL 3, GPIO 4/5). CRSF is 8N1 so any Pi
      // UART works; no parity constraint, no disable-bt.
      m_device = value;
      handled = true;
    }
    else if (param == "telemetry")
    {
      m_telemetry_enabled = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "telem_hz")
    {
      double hz = atof(value.c_str());
      if (hz <= 0 || hz > 10)
        reportConfigWarning("telem_hz out of (0,10]: " + value);
      else
        m_telem_hz = hz;
      handled = true;
    }
    else if (param == "gps_hz")
    {
      // 0 disables the GPS/distance frames.
      double hz = atof(value.c_str());
      if (hz < 0 || hz > 10)
        reportConfigWarning("gps_hz out of [0,10]: " + value);
      else
        m_gps_hz = hz;
      handled = true;
    }
    else if (param == "validated_channels")
    {
      // SBUS-era knob: range-check validation is superseded by
      // the CRSF CRC. Warn instead of failing so a copied SBUS
      // config block still launches during the migration.
      reportConfigWarning("validated_channels is SBUS-only; "
                          "CRSF frames are CRC-validated. Ignored.");
      handled = true;
    }
    else if (param == "debug")
    {
      m_debug = (tolower(value) == "true") ? true : false;
      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, 0, m_fname_buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, 0, m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                m_app_name.c_str(), fmt);
      }
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // First open attempt happens here (before the serial thread
  // exists, so m_fd/m_work are still single-owner) purely to get
  // the config warning out early; the serial thread owns the
  // port and all retries from the moment it starts.
  if (!openPort())
    reportRunWarning("Failed to open " + m_device +
                     " (serial thread will keep retrying)");
  m_work.port_open = (m_fd >= 0);

  m_serial_thread = std::thread(&RCInterface::SerialThreadFunction, this);

  // A fresh epoch per launch: the arbiter rebases its sequence
  // comparison on an epoch change rather than treating a
  // restarted receiver as one whose sequences regressed.
  m_rc_input_epoch = bb::make_epoch("rci");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RCInterface::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // Telemetry sources only; RC input needs no subscriptions.
  Register("NVGR_ROLLING_VOLTAGE", 0);
  Register("NVGR_ROLLING_CURRENT", 0);
  Register("NAV_LAT", 0);
  Register("NAV_LONG", 0);
  Register("NAV_SPEED", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("BB_CMD_AUTHORITY", 0);
  Register("NVGR_STOP_REASON", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()
//
// NO LOCK. Reads only m_report (the app thread's own snapshot
// copy from Iterate) and app-private state. PostReport() calls
// this from inside Iterate — under the old single-mutex design
// that re-entry self-deadlocked on the first iteration.

bool RCInterface::buildReport()
{
  const uint64_t now = microsNow();
  const RcLinkState &ls = m_report.link_state;

  m_msgs << "============================================" << endl;
  m_msgs << "RC Interface Status (CRSF)" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Device: " << m_device
         << (m_report.port_open ? "  (open)" : "  (NOT OPEN, retrying)") << endl;

  m_msgs << "Frame Valid (per-frame):    " << (ls.frameValid()  ? "YES" : "NO") << endl;
  m_msgs << "RC Connected (debounced):   " << (ls.rcConnected() ? "YES" : "NO") << endl << endl;
  m_msgs << "Failsafe (uplink LQ 0):     " << (ls.failsafe()    ? "YES" : "NO") << endl;

  double frame_age_ms = (ls.lastFrameUs() == 0) ? -1.0
    : (now - ls.lastFrameUs()) / 1000.0;
  m_msgs << "Time Since Last Frame: "
         << (frame_age_ms < 0 ? "never" : doubleToStringX(frame_age_ms, 1) + " ms")
         << endl;
  m_msgs << "Frames Received:            " << ls.framesReceived() << endl;
  m_msgs << "Consecutive Good Frames:    " << ls.consecGoodFrames() << endl;
  m_msgs << "CRC Failures:               " << m_report.crc_fails << endl << endl;

  if (m_report.have_link) {
    m_msgs << "Uplink LQ " << (int)m_report.link.uplink_lq
           << "%  RSSI " << m_report.link.uplink_rssi_ant1
           << " dBm  SNR " << (int)m_report.link.uplink_snr
           << " dB  TX " << crsfTxPowerMilliwatts(m_report.link.uplink_tx_power_idx)
           << " mW" << endl;
  } else {
    m_msgs << "No link statistics yet" << endl;
  }

  m_msgs << "Telemetry: "
         << (m_telemetry_enabled ? "enabled" : "disabled")
         << "  sent " << m_report.sent_frames
         << "  dropped " << m_report.dropped_writes
         << "  partial " << m_report.partial_writes << endl;

  // Channel table per Contract v2 (rc_handset_architecture.md).
  // The "Mapped" column shows the decoded state with its meaning
  // so a HIL pass can be read straight off the appcast.
  static const char *kill_names[]    = {"?", "RUNNING",  "KILLED"};
  static const char *mode_names[]    = {"?", "AUTO",     "RC"};
  static const char *action_names[]  = {"?", "HOLD",     "RESUME", "RETURN"};
  static const char *deadman_names[] = {"?", "ENABLED",  "DISABLED"};
  static const char *pwr_names[]     = {"?", "OFF",      "ON"};
  static const char *mark_names[]    = {"?", "-",        "MARK"};

  auto stateStr = [&](int idx, const char **names, int n_states) {
    int s = (int)m_scaled_channels[idx];
    if (s < 1 || s > n_states)
      return intToString(s) + " (?)";
    return intToString(s) + " (" + names[s] + ")";
  };

  ACTable actab(4);
  actab << "CH | Name (control) | Raw | Mapped";
  actab.addHeaderLines();

  actab << "1" << "STEER (right stick L/R)"  << m_report.channels[0] << doubleToStringX(m_scaled_channels[0], 1) + "%";
  actab << "2" << "AUX_AXIS_1 (right stick F/B)" << m_report.channels[1] << doubleToStringX(m_scaled_channels[1], 1) + "%";
  actab << "3" << "THRUST (left stick F/B)"  << m_report.channels[2] << doubleToStringX(m_scaled_channels[2], 1) + "%";
  actab << "4" << "AUX_AXIS_2 (left stick L/R)"  << m_report.channels[3] << doubleToStringX(m_scaled_channels[3], 1) + "%";

  actab << "5"  << "KILL (SW1)"         << m_report.channels[4] << stateStr(4, kill_names, 2);
  actab << "6"  << "MODE (SF)"          << m_report.channels[5] << stateStr(5, mode_names, 2);
  actab << "7"  << "ACTION (SB)"        << m_report.channels[6] << stateStr(6, action_names, 3);
  actab << "8"  << "DEADMAN_EN (SW2)"   << m_report.channels[7] << stateStr(7, deadman_names, 2);
  actab << "9"  << "PAYLOAD_PWR (SW3)"  << m_report.channels[8] << stateStr(8, pwr_names, 2);
  actab << "10" << "PAYLOAD_MODE (SC)"  << m_report.channels[9] << intToString((int)m_scaled_channels[9]);
  actab << "11" << "THRUST_LIMIT (S1)"  << m_report.channels[10] << doubleToStringX(m_scaled_channels[10], 0) + "%";
  actab << "12" << "MARK (SH)"          << m_report.channels[11] << stateStr(11, mark_names, 2);

  for (int i = 12; i < RC_NUM_CHANNELS; i++) {
    actab << intToString(i+1) << "reserved" << m_report.channels[i] << m_report.channels[i];
  }

  m_msgs << actab.getFormattedString();

  return(true);
}
