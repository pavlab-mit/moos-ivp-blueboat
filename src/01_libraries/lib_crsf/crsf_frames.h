/*************************************************************
 * CRSF payload decoding
 *
 * Typed decode of the frame payloads this project consumes.
 * Pure functions over byte buffers - no I/O, no state, no
 * platform dependency. CrsfParser hands us a validated
 * {type, payload}; this layer turns the payload into numbers.
 *
 * Only 0x16 (RC channels) and 0x14 (link statistics) are
 * decoded here. Telemetry ENCODE (0x08 battery, 0x21 flight
 * mode) lands in a later commit.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#ifndef CRSF_FRAMES_H
#define CRSF_FRAMES_H

#include <stdint.h>
#include <stddef.h>
#include <vector>

#include "crsf_parser.h"

//---------------------------------------------------------
// RC channels (0x16)
//
// 16 channels x 11 bits = 176 bits = 22 payload bytes, packed
// LSB-first with no padding. Channel n occupies bits
// [11*n, 11*n+11) of the little-endian bitstream.
//
// ExpressLRS 4.0+ may append one extra arm-status byte, making
// the payload 23 bytes. The channel data is unchanged; the extra
// byte is simply ignored, so both lengths decode identically.

#define CRSF_NUM_CHANNELS          16
#define CRSF_CHANNELS_PAYLOAD_SIZE 22
#define CRSF_CHANNELS_PAYLOAD_SIZE_ARM 23   // ELRS 4.0+ variant

// Raw channel values are 11-bit ticks, [0, 2047].
//
// The canonical calibration points are identical to the SBUS
// values already used across this repo, which is why downstream
// scaling (ScaleJoystick / MapSwitch) carries over unchanged:
//
//   us   = 1500 + (5/8) * (ticks - 992)
//   tick = 992 + (8/5) * (us - 1500)
#define CRSF_TICKS_MIN  172    // ~988 us
#define CRSF_TICKS_MID  992    // 1500 us
#define CRSF_TICKS_MAX  1811   // ~2012 us

struct CrsfChannels {
  uint16_t channel[CRSF_NUM_CHANNELS];
};

// Decode a 0x16 payload. Returns false if the payload is not a
// recognized length, leaving `out` untouched.
bool crsfDecodeChannels(const uint8_t* payload, size_t len,
                        CrsfChannels& out);

inline bool crsfDecodeChannels(const std::vector<uint8_t>& payload,
                               CrsfChannels& out)
{
  return crsfDecodeChannels(payload.data(), payload.size(), out);
}

// Convert an 11-bit tick value to microseconds.
double crsfTicksToMicros(uint16_t ticks);

//---------------------------------------------------------
// Link statistics (0x14)
//
// 10 bytes, fixed layout. This is the frame that makes CRSF
// worth the migration: it reports link health directly instead
// of leaving us to infer it from frame-loss heuristics, and it
// is what would have made zoe's sustained-failsafe condition
// obvious at a glance.
//
// RSSI is transmitted as a POSITIVE magnitude meaning negative
// dBm (wire value 50 => -50 dBm). Decoding stores the true
// signed dBm so consumers never have to remember the convention.

#define CRSF_LINK_STATS_PAYLOAD_SIZE 10

struct CrsfLinkStats {
  int16_t uplink_rssi_ant1;    // dBm (negative)
  int16_t uplink_rssi_ant2;    // dBm (negative)
  uint8_t uplink_lq;           // %, 0-100
  int8_t  uplink_snr;          // dB, signed
  uint8_t active_antenna;      // 0 = ant1, 1 = ant2
  uint8_t rf_mode;             // band-dependent enum
  uint8_t uplink_tx_power_idx; // index into the power table
  // CAUTION: ELRS receivers never populate the downlink_* fields in
  // a normal build (rx_main.cpp writes them only under
  // DEBUG_BF_LINK_STATS), so they read 0 regardless of telemetry
  // health. Downlink quality is measured at the TX module and only
  // surfaces as the handset TQly/TRSS/TSNR sensors. Decoded here for
  // wire completeness; do not publish or alarm on them.
  int16_t downlink_rssi;       // dBm (negative)
  uint8_t downlink_lq;         // %, 0-100
  int8_t  downlink_snr;        // dB, signed
};

// Decode a 0x14 payload. Returns false on wrong length.
bool crsfDecodeLinkStats(const uint8_t* payload, size_t len,
                         CrsfLinkStats& out);

inline bool crsfDecodeLinkStats(const std::vector<uint8_t>& payload,
                                CrsfLinkStats& out)
{
  return crsfDecodeLinkStats(payload.data(), payload.size(), out);
}

// TX power in mW for a power index, or 0 if the index is unknown.
uint16_t crsfTxPowerMilliwatts(uint8_t index);

//---------------------------------------------------------
// Telemetry ENCODE (vehicle -> receiver -> handset)
//
// Header form: both frames below are BROADCAST frames with the
// SIMPLE header - [sync][len][type][payload][crc8], no
// destination/origin bytes.
//
// This is worth stating explicitly because it is easy to get
// wrong: the CRSF wiki's frame-type table marks 0x08 and 0x21
// with "Extended: Y", but the authoritative TBS spec says
// "frames with type 0x28 and higher have extended header" and
// lists both of these as simple. Adding address bytes would
// produce frames the receiver silently drops - a failure with no
// error message anywhere.
//
// Sync byte: a flight controller sends telemetry with 0xC8,
// which doubles as the FC device address.

#define CRSF_ADDRESS_BROADCAST       0x00
#define CRSF_ADDRESS_FLIGHT_CTRL     0xC8
#define CRSF_ADDRESS_CRSF_RECEIVER   0xEC
#define CRSF_ADDRESS_CRSF_TX         0xEE
#define CRSF_ADDRESS_REMOTE_CONTROL  0xEA

// Battery sensor (0x08) payload, 8 bytes:
//   [0..1] voltage, int16 BIG-endian, decivolts (25.2 V -> 252)
//   [2..3] current, int16 BIG-endian, deciamps  (18.9 A -> 189)
//   [4..6] used capacity, 24-bit BIG-endian, mAh
//   [7]    remaining, uint8, percent
#define CRSF_BATTERY_PAYLOAD_SIZE 8

// Flight mode (0x21): null-terminated string, max 14 bytes
// INCLUDING the null (13 characters of text).
#define CRSF_FLIGHT_MODE_MAX_LEN 14

// Build a complete battery frame into `out` (sync..crc inclusive).
// Returns the number of bytes written.
size_t crsfEncodeBattery(double volts, double amps,
                         uint32_t used_mah, uint8_t remaining_pct,
                         std::vector<uint8_t>& out);

// Build a complete flight-mode frame. The string is truncated to
// 13 characters plus a null if longer. Returns bytes written.
size_t crsfEncodeFlightMode(const char* mode, std::vector<uint8_t>& out);

// GPS (0x02) payload, 15 bytes, all BIG-endian (telemetry
// convention, same as the battery frame):
//   [0..3]   latitude,    int32,  degrees * 1e7
//   [4..7]   longitude,   int32,  degrees * 1e7
//   [8..9]   groundspeed, uint16, (km/h) * 10
//   [10..11] heading,     uint16, degrees * 100
//   [12..13] altitude,    uint16, meters with a +1000 m offset
//   [14]     satellites,  uint8
//
// EdgeTX auto-discovers this frame as the GPS / GSpd / Hdg / Alt /
// Sats sensors and can derive distance-to-vehicle from it.
#define CRSF_GPS_PAYLOAD_SIZE 15

// Build a complete GPS frame. Inputs use vehicle-native units
// (degrees, m/s, degrees true, meters); unit conversion to the
// wire format happens here, in exactly one place. Values are
// clamped to their wire ranges. Returns bytes written.
size_t crsfEncodeGps(double lat_deg, double lon_deg,
                     double speed_mps, double heading_deg,
                     double alt_m, uint8_t sats,
                     std::vector<uint8_t>& out);

// Custom telemetry (0x80) -----------------------------------
//
// 0x80 is the CRSF "custom telemetry" frame type, the same one
// ArduPilot uses for its passthrough sensors. It is NOT decoded
// natively by EdgeTX; instead ELRS tunnels it over the air and
// EdgeTX queues it for Lua (crossfireTelemetryPop), where a
// small decoder script publishes it as a named sensor. This is
// the sanctioned way to carry values CRSF has no frame for,
// without mislabeling a standard field.
//
// Payload convention (ours): [subtype][data...], so several
// custom values can share the frame type.
//   0xB1 DISTANCE: uint16 BIG-endian, meters from the MOOS
//        origin (LatOrigin/LongOrigin), i.e. hypot(NAV_X,NAV_Y).
//        Decoded by SCRIPTS/MIXES/dstrx.lua -> sensor "Dst".
#define CRSF_FRAMETYPE_CUSTOM_TELEM 0x80
#define CRSF_CUSTOM_SUBTYPE_DISTANCE 0xB1

// Build a custom distance frame. Meters clamped to [0, 65535].
size_t crsfEncodeCustomDistance(double meters,
                                std::vector<uint8_t>& out);

// Assemble any simple-header frame: [sync][len][type][payload][crc].
// Returns bytes written, or 0 if the payload is too large.
size_t crsfEncodeFrame(uint8_t type, const uint8_t* payload, size_t len,
                       std::vector<uint8_t>& out);

#endif /* CRSF_FRAMES_H */
