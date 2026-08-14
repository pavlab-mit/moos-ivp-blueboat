/*************************************************************
 * Unit tests for CRSF payload decode (0x16 channels, 0x14 link
 * statistics).
 *
 * Host-side only: no hardware, no serial port, no MOOS.
 *
 * The channel unpack is validated against an INDEPENDENT
 * bit-level reference written below - the protocol definition
 * transcribed directly, not a second copy of the library's loop.
 * It is deliberately NOT checked against lib_sbus: that code is
 * untested and being retired, so agreement with it would prove
 * only that both share the same behaviour, silently ratifying
 * any latent bit-order bug.
 *
 * Build:  ./build.sh --unit_tests
 * Run:    ./bin/test_crsf_frames
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "crsf_frames.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

//---------------------------------------------------------
// Minimal check harness (same convention as test_crsf_parser)

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) {
    g_failures++;
    fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

static void checkEq(long long got, long long want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %lld, want %lld)\n",
            what.c_str(), got, want);
  }
}

static void checkNear(double got, double want, double tol,
                      const std::string& what)
{
  g_checks++;
  if (std::fabs(got - want) > tol) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %.4f, want %.4f)\n",
            what.c_str(), got, want);
  }
}

//---------------------------------------------------------
// Independent reference implementations
//
// Written from the spec, intentionally in a different style from
// the library so a shared mistake is unlikely.

// PACK: place 16 x 11-bit values into 22 bytes, LSB-first.
// Used to build test payloads.
static std::vector<uint8_t> refPackChannels(const uint16_t ch[16])
{
  std::vector<uint8_t> bytes(CRSF_CHANNELS_PAYLOAD_SIZE, 0);

  int bitpos = 0;
  for (int c = 0; c < 16; c++) {
    for (int b = 0; b < 11; b++) {
      if ((ch[c] >> b) & 0x01)
        bytes[bitpos / 8] |= (uint8_t)(1u << (bitpos % 8));
      bitpos++;
    }
  }
  return bytes;
}

// UNPACK reference: accumulate a 32-bit sliding window, a
// structurally different approach from the library's per-bit
// indexing, so the two agreeing is meaningful.
static void refUnpackChannels(const uint8_t* p, uint16_t out[16])
{
  uint32_t acc  = 0;
  int      bits = 0;
  int      idx  = 0;

  for (int c = 0; c < 16; c++) {
    while (bits < 11) {
      acc |= (uint32_t)p[idx++] << bits;
      bits += 8;
    }
    out[c] = (uint16_t)(acc & 0x07FF);
    acc  >>= 11;
    bits  -= 11;
  }
}

//---------------------------------------------------------
// Channel decode: agreement with the independent reference

static void testChannelsAgainstReference()
{
  struct Case {
    const char* name;
    uint16_t ch[16];
  };

  std::vector<Case> cases;

  // All zero.
  { Case c = {"all zero", {0}}; cases.push_back(c); }

  // All max (11-bit).
  { Case c = {"all 2047", {0}};
    for (int i = 0; i < 16; i++) c.ch[i] = 2047;
    cases.push_back(c); }

  // All centre.
  { Case c = {"all mid", {0}};
    for (int i = 0; i < 16; i++) c.ch[i] = CRSF_TICKS_MID;
    cases.push_back(c); }

  // Distinct value per channel - catches shift-order and
  // off-by-one errors that uniform values cannot. Step chosen so
  // ch16 stays inside the 11-bit range (100 + 15*129 = 2035).
  { Case c = {"distinct per channel", {0}};
    for (int i = 0; i < 16; i++) c.ch[i] = (uint16_t)(100 + i * 129);
    cases.push_back(c); }

  // Alternating extremes - maximum bit churn across boundaries.
  { Case c = {"alternating 0/2047", {0}};
    for (int i = 0; i < 16; i++) c.ch[i] = (i % 2) ? 2047 : 0;
    cases.push_back(c); }

  // Canonical stick values.
  { Case c = {"min/mid/max mix", {0}};
    for (int i = 0; i < 16; i++) {
      if (i % 3 == 0)      c.ch[i] = CRSF_TICKS_MIN;
      else if (i % 3 == 1) c.ch[i] = CRSF_TICKS_MID;
      else                 c.ch[i] = CRSF_TICKS_MAX;
    }
    cases.push_back(c); }

  for (const auto& tc : cases) {
    // Guard the test data itself: a vector above 2047 would be
    // silently truncated to 11 bits, and the round-trip check
    // would then fail for a reason that has nothing to do with
    // the decoder. Catch bad vectors explicitly instead.
    for (int i = 0; i < 16; i++)
      check(tc.ch[i] <= 2047,
            std::string(tc.name) + ": test vector ch" +
            std::to_string(i + 1) + " fits in 11 bits");

    auto payload = refPackChannels(tc.ch);

    CrsfChannels got;
    bool ok = crsfDecodeChannels(payload, got);
    check(ok, std::string(tc.name) + ": decode returned true");
    if (!ok) continue;

    uint16_t want[16];
    refUnpackChannels(payload.data(), want);

    for (int i = 0; i < 16; i++) {
      checkEq(got.channel[i], want[i],
              std::string(tc.name) + ": ch" + std::to_string(i + 1) +
              " matches reference");
      checkEq(got.channel[i], tc.ch[i],
              std::string(tc.name) + ": ch" + std::to_string(i + 1) +
              " round-trips");
    }
  }
}

//---------------------------------------------------------
// Single-channel isolation
//
// For each channel in turn: set it to a distinctive value, leave
// all others zero, and confirm exactly that one channel carries
// it. This is the test that catches "channel N reads channel
// N+1's bits" - the classic packed-bitfield bug, and one that
// uniform test vectors cannot detect.

static void testChannelIsolation()
{
  for (int target = 0; target < 16; target++) {
    uint16_t ch[16] = {0};
    ch[target] = 0x5AB;            // 0b101_1010_1011, asymmetric

    auto payload = refPackChannels(ch);

    CrsfChannels got;
    if (!crsfDecodeChannels(payload, got)) {
      check(false, "isolation ch" + std::to_string(target + 1) + ": decode");
      continue;
    }

    for (int i = 0; i < 16; i++) {
      uint16_t want = (i == target) ? 0x5AB : 0;
      checkEq(got.channel[i], want,
              "isolation ch" + std::to_string(target + 1) +
              ": ch" + std::to_string(i + 1));
    }
  }

  // Walking single bit through every bit of every channel.
  for (int target = 0; target < 16; target++) {
    for (int bit = 0; bit < 11; bit++) {
      uint16_t ch[16] = {0};
      ch[target] = (uint16_t)(1u << bit);

      auto payload = refPackChannels(ch);
      CrsfChannels got;
      if (!crsfDecodeChannels(payload, got)) continue;

      bool clean = true;
      for (int i = 0; i < 16; i++) {
        uint16_t want = (i == target) ? (uint16_t)(1u << bit) : 0;
        if (got.channel[i] != want) clean = false;
      }
      check(clean, "walking bit: ch" + std::to_string(target + 1) +
                   " bit " + std::to_string(bit));
    }
  }
}

//---------------------------------------------------------
// Payload length handling

static void testChannelPayloadLengths()
{
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(200 + i * 100);
  auto payload = refPackChannels(ch);

  // Canonical 22-byte payload.
  {
    CrsfChannels got;
    check(crsfDecodeChannels(payload, got), "22-byte payload accepted");
  }

  // ELRS 4.0+ 23-byte variant: extra arm byte, same channel data.
  {
    auto p23 = payload;
    p23.push_back(0x01);

    CrsfChannels got23, got22;
    check(crsfDecodeChannels(p23, got23), "23-byte payload accepted");
    crsfDecodeChannels(payload, got22);

    bool same = true;
    for (int i = 0; i < 16; i++)
      if (got23.channel[i] != got22.channel[i]) same = false;
    check(same, "23-byte payload decodes identically to 22-byte");
  }

  // Wrong lengths rejected.
  for (size_t len : {(size_t)0, (size_t)1, (size_t)21, (size_t)24, (size_t)60}) {
    std::vector<uint8_t> bad(len, 0x00);
    CrsfChannels got;
    check(!crsfDecodeChannels(bad, got),
          "payload length " + std::to_string(len) + " rejected");
  }

  // Null pointer must not crash.
  {
    CrsfChannels got;
    check(!crsfDecodeChannels(nullptr, 22, got), "null payload rejected");
  }
}

//---------------------------------------------------------
// Ticks -> microseconds
//
// The calibration points that matter operationally. These are the
// same raw values lib_sbus uses (172/992/1812), which is why the
// existing downstream scaling carries over unchanged.

static void testTicksToMicros()
{
  checkNear(crsfTicksToMicros(CRSF_TICKS_MID), 1500.0, 0.001, "992 ticks = 1500us");
  checkNear(crsfTicksToMicros(CRSF_TICKS_MIN),  987.5, 0.5,   "172 ticks ~= 988us");
  checkNear(crsfTicksToMicros(CRSF_TICKS_MAX), 2011.875, 0.5, "1811 ticks ~= 2012us");

  // Linearity and monotonicity across the range.
  double prev = crsfTicksToMicros(0);
  for (uint16_t t = 1; t <= 2047; t++) {
    double us = crsfTicksToMicros(t);
    if (us <= prev) {
      check(false, "ticks->us monotonic at " + std::to_string(t));
      break;
    }
    prev = us;
  }
  check(true, "ticks->us monotonic across full range");

  // Symmetry about centre.
  double below = crsfTicksToMicros(CRSF_TICKS_MID - 320);
  double above = crsfTicksToMicros(CRSF_TICKS_MID + 320);
  checkNear(1500.0 - below, above - 1500.0, 0.001, "ticks->us symmetric about mid");
}

//---------------------------------------------------------
// Link statistics decode

static void testLinkStats()
{
  // Representative healthy link.
  {
    std::vector<uint8_t> p = {
      45,        // uplink RSSI ant1 -> -45 dBm
      52,        // uplink RSSI ant2 -> -52 dBm
      100,       // uplink LQ 100%
      (uint8_t)(int8_t)12,   // uplink SNR +12 dB
      0,         // active antenna 1
      2,         // RF mode
      3,         // TX power idx -> 100 mW
      38,        // downlink RSSI -> -38 dBm
      99,        // downlink LQ 99%
      (uint8_t)(int8_t)8     // downlink SNR +8 dB
    };

    CrsfLinkStats s;
    check(crsfDecodeLinkStats(p, s), "link stats: decode ok");

    checkEq(s.uplink_rssi_ant1, -45, "link stats: uplink RSSI ant1 negated");
    checkEq(s.uplink_rssi_ant2, -52, "link stats: uplink RSSI ant2 negated");
    checkEq(s.uplink_lq, 100, "link stats: uplink LQ");
    checkEq(s.uplink_snr, 12, "link stats: uplink SNR");
    checkEq(s.active_antenna, 0, "link stats: active antenna");
    checkEq(s.rf_mode, 2, "link stats: rf mode");
    checkEq(s.uplink_tx_power_idx, 3, "link stats: tx power index");
    checkEq(s.downlink_rssi, -38, "link stats: downlink RSSI negated");
    checkEq(s.downlink_lq, 99, "link stats: downlink LQ");
    checkEq(s.downlink_snr, 8, "link stats: downlink SNR");
  }

  // NEGATIVE SNR - the sign-handling case that is easy to get
  // wrong and produces plausible-but-wrong numbers when it is.
  // A marginal link is exactly when SNR goes negative, so this
  // matters precisely when the data matters most.
  {
    std::vector<uint8_t> p(10, 0);
    p[3] = (uint8_t)(int8_t)(-7);
    p[9] = (uint8_t)(int8_t)(-20);

    CrsfLinkStats s;
    check(crsfDecodeLinkStats(p, s), "negative SNR: decode ok");
    checkEq(s.uplink_snr,   -7,  "negative uplink SNR decoded signed");
    checkEq(s.downlink_snr, -20, "negative downlink SNR decoded signed");
  }

  // Weak-link RSSI: large magnitude stays negative, not wrapped.
  {
    std::vector<uint8_t> p(10, 0);
    p[0] = 123;   // -123 dBm, ELRS 25Hz sensitivity floor
    p[7] = 130;

    CrsfLinkStats s;
    check(crsfDecodeLinkStats(p, s), "weak link: decode ok");
    checkEq(s.uplink_rssi_ant1, -123, "weak link: -123 dBm");
    checkEq(s.downlink_rssi,    -130, "weak link: -130 dBm not wrapped");
  }

  // Total link loss: LQ 0.
  {
    std::vector<uint8_t> p(10, 0);
    CrsfLinkStats s;
    check(crsfDecodeLinkStats(p, s), "zero payload: decode ok");
    checkEq(s.uplink_lq, 0, "link loss: uplink LQ 0");
    checkEq(s.downlink_lq, 0, "link loss: downlink LQ 0");
  }

  // Wrong lengths rejected.
  for (size_t len : {(size_t)0, (size_t)9, (size_t)11, (size_t)22}) {
    std::vector<uint8_t> bad(len, 0);
    CrsfLinkStats s;
    check(!crsfDecodeLinkStats(bad, s),
          "link stats length " + std::to_string(len) + " rejected");
  }

  {
    CrsfLinkStats s;
    check(!crsfDecodeLinkStats(nullptr, 10, s), "null link stats rejected");
  }
}

//---------------------------------------------------------
// TX power table

static void testTxPower()
{
  checkEq(crsfTxPowerMilliwatts(0), 0,    "tx power 0 -> 0mW");
  checkEq(crsfTxPowerMilliwatts(1), 10,   "tx power 1 -> 10mW");
  checkEq(crsfTxPowerMilliwatts(2), 25,   "tx power 2 -> 25mW");
  checkEq(crsfTxPowerMilliwatts(3), 100,  "tx power 3 -> 100mW");
  checkEq(crsfTxPowerMilliwatts(4), 500,  "tx power 4 -> 500mW");
  checkEq(crsfTxPowerMilliwatts(5), 1000, "tx power 5 -> 1000mW");
  checkEq(crsfTxPowerMilliwatts(6), 2000, "tx power 6 -> 2000mW");
  checkEq(crsfTxPowerMilliwatts(7), 50,   "tx power 7 -> 50mW (out of order)");
  checkEq(crsfTxPowerMilliwatts(200), 0,  "unknown tx power index -> 0");
}

//---------------------------------------------------------
// End-to-end: parser -> frames
//
// Confirms the two layers compose, using the real parser rather
// than hand-fed payloads.

static void testParserToFrames()
{
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(172 + i * 100);
  auto payload = refPackChannels(ch);

  // Build a real 0x16 frame.
  std::vector<uint8_t> frame;
  frame.push_back(CRSF_SYNC_BYTE);
  frame.push_back((uint8_t)(payload.size() + 2));
  frame.push_back(CRSF_FRAMETYPE_RC_CHANNELS);
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(crsfCrc8(&frame[2], payload.size() + 1));

  CrsfParser parser;
  std::vector<CrsfFrame> out;
  parser.feed(frame.data(), frame.size(), out);

  checkEq(out.size(), 1, "end-to-end: one frame parsed");
  if (out.size() != 1) return;
  checkEq(out[0].type, CRSF_FRAMETYPE_RC_CHANNELS, "end-to-end: type");

  CrsfChannels got;
  check(crsfDecodeChannels(out[0].payload, got), "end-to-end: channels decode");
  for (int i = 0; i < 16; i++)
    checkEq(got.channel[i], ch[i],
            "end-to-end: ch" + std::to_string(i + 1));

  // And a link-stats frame through the same path.
  std::vector<uint8_t> ls = {45, 50, 100, (uint8_t)(int8_t)10,
                             0, 2, 3, 40, 98, (uint8_t)(int8_t)(-3)};
  std::vector<uint8_t> lf;
  lf.push_back(CRSF_SYNC_BYTE);
  lf.push_back((uint8_t)(ls.size() + 2));
  lf.push_back(CRSF_FRAMETYPE_LINK_STATISTICS);
  lf.insert(lf.end(), ls.begin(), ls.end());
  lf.push_back(crsfCrc8(&lf[2], ls.size() + 1));

  out.clear();
  parser.feed(lf.data(), lf.size(), out);
  checkEq(out.size(), 1, "end-to-end: link stats frame parsed");
  if (out.size() == 1) {
    CrsfLinkStats s;
    check(crsfDecodeLinkStats(out[0].payload, s), "end-to-end: link stats decode");
    checkEq(s.uplink_lq, 100, "end-to-end: uplink LQ");
    checkEq(s.downlink_snr, -3, "end-to-end: negative downlink SNR");
  }
}

//---------------------------------------------------------
// Telemetry encode
//
// Every encoded frame is fed back through the REAL parser: if a
// frame we build cannot be parsed by the same code that reads the
// receiver, it would be silently dropped on the wire with no
// error anywhere. Round-tripping is the only way to catch that
// without hardware.

static void testEncodeBattery()
{
  std::vector<uint8_t> f;
  size_t n = crsfEncodeBattery(25.2, 18.9, 2199, 100, f);

  checkEq(n, f.size(), "battery: returned size matches vector");
  checkEq(f.size(), 3 + CRSF_BATTERY_PAYLOAD_SIZE + 1,
          "battery: total frame size");
  checkEq(f[0], CRSF_ADDRESS_FLIGHT_CTRL, "battery: sync is 0xC8");
  checkEq(f[1], CRSF_BATTERY_PAYLOAD_SIZE + 2, "battery: len field");
  checkEq(f[2], CRSF_FRAMETYPE_BATTERY_SENSOR, "battery: type 0x08");

  // Big-endian decivolts: 25.2 V -> 252 -> 0x00FC.
  checkEq(f[3], 0x00, "battery: voltage high byte");
  checkEq(f[4], 0xFC, "battery: voltage low byte (252 dV)");
  // 18.9 A -> 189 -> 0x00BD
  checkEq(f[5], 0x00, "battery: current high byte");
  checkEq(f[6], 0xBD, "battery: current low byte (189 dA)");
  // 2199 mAh -> 0x000897
  checkEq(f[7], 0x00, "battery: capacity byte 0");
  checkEq(f[8], 0x08, "battery: capacity byte 1");
  checkEq(f[9], 0x97, "battery: capacity byte 2");
  checkEq(f[10], 100, "battery: remaining percent");

  // Must parse with the real parser.
  CrsfParser p;
  std::vector<CrsfFrame> out;
  p.feed(f.data(), f.size(), out);
  checkEq(out.size(), 1, "battery: round-trips through parser");
  checkEq(p.framesCrcFail(), 0, "battery: CRC valid");
  if (out.size() == 1) {
    checkEq(out[0].type, CRSF_FRAMETYPE_BATTERY_SENSOR, "battery: parsed type");
    checkEq(out[0].payload.size(), CRSF_BATTERY_PAYLOAD_SIZE,
            "battery: parsed payload size");
  }

  // Rounding, not truncation: 12.35 V -> 124 dV (not 123).
  {
    std::vector<uint8_t> g;
    crsfEncodeBattery(12.35, 0.0, 0, 0, g);
    uint16_t dv = (uint16_t)((g[3] << 8) | g[4]);
    checkEq(dv, 124, "battery: 12.35V rounds to 124 dV");
  }

  // A realistic BlueBoat pack voltage.
  {
    std::vector<uint8_t> g;
    crsfEncodeBattery(24.6, 3.2, 0, 78, g);
    uint16_t dv = (uint16_t)((g[3] << 8) | g[4]);
    uint16_t da = (uint16_t)((g[5] << 8) | g[6]);
    checkEq(dv, 246, "battery: 24.6V -> 246 dV");
    checkEq(da,  32, "battery: 3.2A -> 32 dA");
    checkEq(g[10], 78, "battery: 78 percent");
  }

  // Clamping: absurd values must not wrap into plausible ones.
  {
    std::vector<uint8_t> g;
    crsfEncodeBattery(1e6, 1e6, 0xFFFFFFFF, 250, g);
    int16_t dv = (int16_t)((g[3] << 8) | g[4]);
    checkEq(dv, 32767, "battery: voltage clamps to int16 max");
    checkEq(g[7], 0xFF, "battery: capacity clamps to 24-bit");
    checkEq(g[10], 100, "battery: percent clamps to 100");
  }

  // Negative current (regenerative / reversed sensor) survives.
  {
    std::vector<uint8_t> g;
    crsfEncodeBattery(24.0, -5.0, 0, 50, g);
    int16_t da = (int16_t)((g[5] << 8) | g[6]);
    checkEq(da, -50, "battery: negative current encodes signed");
  }
}

static void testEncodeFlightMode()
{
  std::vector<uint8_t> f;
  size_t n = crsfEncodeFlightMode("ACRO", f);

  checkEq(n, f.size(), "flight mode: returned size");
  checkEq(f[0], CRSF_ADDRESS_FLIGHT_CTRL, "flight mode: sync 0xC8");
  checkEq(f[2], CRSF_FRAMETYPE_FLIGHT_MODE, "flight mode: type 0x21");
  // "ACRO" + null = 5 payload bytes -> 41 43 52 4F 00
  checkEq(f[3], 0x41, "flight mode: 'A'");
  checkEq(f[4], 0x43, "flight mode: 'C'");
  checkEq(f[5], 0x52, "flight mode: 'R'");
  checkEq(f[6], 0x4F, "flight mode: 'O'");
  checkEq(f[7], 0x00, "flight mode: null terminator");

  CrsfParser p;
  std::vector<CrsfFrame> out;
  p.feed(f.data(), f.size(), out);
  checkEq(out.size(), 1, "flight mode: round-trips through parser");
  checkEq(p.framesCrcFail(), 0, "flight mode: CRC valid");

  // The strings this project will actually send.
  for (const char* m : {"RC", "AUTO", "MANUAL", "DEADMAN", "PARK"}) {
    std::vector<uint8_t> g;
    crsfEncodeFlightMode(m, g);

    CrsfParser pp;
    std::vector<CrsfFrame> oo;
    pp.feed(g.data(), g.size(), oo);
    check(oo.size() == 1,
          std::string("flight mode '") + m + "': parses");
    if (oo.size() == 1) {
      std::string got((const char*)oo[0].payload.data());
      check(got == m, std::string("flight mode '") + m + "': text round-trips");
    }
  }

  // Over-long string truncates to 13 chars + null, and must still
  // be a legal parseable frame rather than overflowing.
  {
    std::vector<uint8_t> g;
    crsfEncodeFlightMode("THIS_IS_A_VERY_LONG_MODE_NAME", g);

    CrsfParser pp;
    std::vector<CrsfFrame> oo;
    pp.feed(g.data(), g.size(), oo);
    checkEq(oo.size(), 1, "flight mode: over-long string still parses");
    if (oo.size() == 1) {
      checkEq(oo[0].payload.size(), CRSF_FLIGHT_MODE_MAX_LEN,
              "flight mode: truncated to 14 bytes");
      checkEq(oo[0].payload.back(), 0x00,
              "flight mode: truncated string still null-terminated");
    }
  }

  // Empty string: just a null.
  {
    std::vector<uint8_t> g;
    crsfEncodeFlightMode("", g);
    CrsfParser pp;
    std::vector<CrsfFrame> oo;
    pp.feed(g.data(), g.size(), oo);
    checkEq(oo.size(), 1, "flight mode: empty string parses");
    if (oo.size() == 1)
      checkEq(oo[0].payload.size(), 1, "flight mode: empty is one null byte");
  }

  // Null pointer rejected.
  {
    std::vector<uint8_t> g;
    checkEq(crsfEncodeFlightMode(nullptr, g), 0, "flight mode: null rejected");
  }
}

static void testEncodeFrameLimits()
{
  // Max legal payload.
  {
    std::vector<uint8_t> pl(CRSF_MAX_PAYLOAD, 0x42), f;
    size_t n = crsfEncodeFrame(0x08, pl.data(), pl.size(), f);
    check(n > 0, "encode: max payload accepted");
    checkEq(f.size(), CRSF_MAX_FRAME_SIZE, "encode: max frame is 64 bytes");

    CrsfParser p;
    std::vector<CrsfFrame> out;
    p.feed(f.data(), f.size(), out);
    checkEq(out.size(), 1, "encode: max frame parses");
  }

  // Over-long payload rejected rather than truncated silently.
  {
    std::vector<uint8_t> pl(CRSF_MAX_PAYLOAD + 1, 0x42), f;
    checkEq(crsfEncodeFrame(0x08, pl.data(), pl.size(), f), 0,
            "encode: over-long payload rejected");
  }

  // Zero-length payload is legal.
  {
    std::vector<uint8_t> f;
    size_t n = crsfEncodeFrame(0x21, nullptr, 0, f);
    check(n > 0, "encode: empty payload accepted");
    CrsfParser p;
    std::vector<CrsfFrame> out;
    p.feed(f.data(), f.size(), out);
    checkEq(out.size(), 1, "encode: empty-payload frame parses");
  }

  // A stream of mixed telemetry frames must all parse back, in
  // order - this is what the TX path will actually emit.
  {
    std::vector<uint8_t> stream;
    for (int i = 0; i < 20; i++) {
      std::vector<uint8_t> b, m;
      crsfEncodeBattery(24.0 + i * 0.1, 2.0, i * 10, (uint8_t)(100 - i), b);
      crsfEncodeFlightMode((i % 2) ? "RC" : "AUTO", m);
      stream.insert(stream.end(), b.begin(), b.end());
      stream.insert(stream.end(), m.begin(), m.end());
    }

    CrsfParser p;
    std::vector<CrsfFrame> out;
    p.feed(stream.data(), stream.size(), out);
    checkEq(out.size(), 40, "encode: mixed telemetry stream all parses");
    checkEq(p.framesCrcFail(), 0, "encode: no CRC failures in stream");
    checkEq(p.bytesDiscarded(), 0, "encode: no bytes discarded");
  }
}

//---------------------------------------------------------
// main()

int main()
{
  printf("CRSF frames unit tests\n");
  printf("======================\n");

  testChannelsAgainstReference();
  testChannelIsolation();
  testChannelPayloadLengths();
  testTicksToMicros();
  testLinkStats();
  testTxPower();
  testParserToFrames();
  testEncodeBattery();
  testEncodeFlightMode();
  testEncodeFrameLimits();

  printf("\n%d checks run\n", g_checks);

  if (g_failures > 0) {
    fprintf(stderr, "\nFAIL: %d of %d checks failed\n", g_failures, g_checks);
    return 1;
  }

  printf("PASS\n");
  return 0;
}
