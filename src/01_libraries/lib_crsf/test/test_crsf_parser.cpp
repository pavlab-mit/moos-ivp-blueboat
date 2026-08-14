/*************************************************************
 * Unit tests for CrsfParser - framing, CRC, resync.
 *
 * Host-side only: no hardware, no serial port, no MOOS. Runs on
 * a laptop. Hardware validation is rc_probe's job, not this
 * file's.
 *
 * Convention follows navigator-cpp: print "FAIL: ..." and return
 * non-zero on failure, "PASS" and 0 on success. No test
 * framework dependency.
 *
 * Build:  ./build.sh --unit_tests
 * Run:    ./bin/test_crsf_parser
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "crsf_parser.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

//---------------------------------------------------------
// Minimal check harness

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

static void checkEq(uint64_t got, uint64_t want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %llu, want %llu)\n",
            what.c_str(),
            (unsigned long long)got,
            (unsigned long long)want);
  }
}

//---------------------------------------------------------
// Frame construction helper
//
// Builds a well-formed frame: [sync][len][type][payload][crc].
// len covers type + payload + crc.

static std::vector<uint8_t> makeFrame(uint8_t type,
                                      const std::vector<uint8_t>& payload,
                                      uint8_t sync = CRSF_SYNC_BYTE)
{
  std::vector<uint8_t> f;
  f.push_back(sync);
  f.push_back((uint8_t)(payload.size() + 2));  // type + payload + crc
  f.push_back(type);
  f.insert(f.end(), payload.begin(), payload.end());

  // CRC covers type + payload, which now sit at f[2..].
  f.push_back(crsfCrc8(&f[2], payload.size() + 1));
  return f;
}

// A representative RC_CHANNELS frame: 22 payload bytes, the real
// size of a packed 16x11-bit channel frame.
static std::vector<uint8_t> makeRcFrame(uint8_t fill = 0xAB)
{
  return makeFrame(CRSF_FRAMETYPE_RC_CHANNELS,
                   std::vector<uint8_t>(22, fill));
}

static size_t feedVec(CrsfParser& p, const std::vector<uint8_t>& bytes,
                      std::vector<CrsfFrame>& out)
{
  return p.feed(bytes.data(), bytes.size(), out);
}

//---------------------------------------------------------
// CRC8 correctness
//
// Independent reimplementation, so a copy-paste error in the
// library doesn't validate itself.

static void testCrc8()
{
  auto refCrc = [](const std::vector<uint8_t>& d) -> uint8_t {
    uint8_t crc = 0;
    for (uint8_t b : d) {
      crc ^= b;
      for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5)
                           : (uint8_t)(crc << 1);
    }
    return crc;
  };

  std::vector<std::vector<uint8_t>> cases = {
    {},
    {0x00},
    {0xFF},
    {0x16, 0x01, 0x02, 0x03},
    std::vector<uint8_t>(60, 0x5A),
  };

  for (size_t i = 0; i < cases.size(); i++) {
    uint8_t got  = crsfCrc8(cases[i].data(), cases[i].size());
    uint8_t want = refCrc(cases[i]);
    checkEq(got, want, "crc8 case " + std::to_string(i));
  }

  // Empty input must be zero, and CRC must actually depend on
  // input order (a plain XOR sum would pass the cases above).
  checkEq(crsfCrc8(nullptr, 0), 0, "crc8 of empty is 0");

  std::vector<uint8_t> ab = {0x01, 0x02};
  std::vector<uint8_t> ba = {0x02, 0x01};
  check(crsfCrc8(ab.data(), 2) != crsfCrc8(ba.data(), 2),
        "crc8 is order-dependent");
}

//---------------------------------------------------------
// A single clean frame decodes

static void testSingleFrame()
{
  CrsfParser p;
  std::vector<CrsfFrame> out;

  std::vector<uint8_t> payload = {0x11, 0x22, 0x33};
  auto f = makeFrame(CRSF_FRAMETYPE_BATTERY_SENSOR, payload);

  size_t n = feedVec(p, f, out);

  checkEq(n, 1, "single frame: one frame emitted");
  checkEq(out.size(), 1, "single frame: out size");
  if (out.size() == 1) {
    checkEq(out[0].type, CRSF_FRAMETYPE_BATTERY_SENSOR, "single frame: type");
    checkEq(out[0].payload.size(), 3, "single frame: payload size");
    check(out[0].payload == payload, "single frame: payload contents");
  }
  checkEq(p.framesValid(), 1, "single frame: valid counter");
  checkEq(p.framesCrcFail(), 0, "single frame: no crc failures");
  checkEq(p.bytesDiscarded(), 0, "single frame: nothing discarded");
  checkEq(p.pending(), 0, "single frame: buffer drained");
}

//---------------------------------------------------------
// Both sync bytes accepted

static void testSyncVariants()
{
  for (uint8_t sync : {(uint8_t)CRSF_SYNC_BYTE, (uint8_t)CRSF_SYNC_BYTE_EDGE}) {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    auto f = makeFrame(CRSF_FRAMETYPE_LINK_STATISTICS,
                       std::vector<uint8_t>(10, 0x07), sync);
    size_t n = feedVec(p, f, out);
    checkEq(n, 1, "sync 0x" + std::to_string((int)sync) + ": frame accepted");
  }
}

//---------------------------------------------------------
// Chunk-boundary independence
//
// The highest-value test here. A real UART splits reads at
// arbitrary offsets, so the parser must produce identical output
// no matter where the chunk boundary lands. Bugs of this shape
// are nondeterministic on hardware and very hard to reproduce on
// a boat.

static void testChunkBoundaries()
{
  auto frame = makeRcFrame();

  // Every possible single split point.
  for (size_t split = 0; split <= frame.size(); split++) {
    CrsfParser p;
    std::vector<CrsfFrame> out;

    p.feed(frame.data(), split, out);
    p.feed(frame.data() + split, frame.size() - split, out);

    if (out.size() != 1) {
      check(false, "split at " + std::to_string(split) + ": expected 1 frame");
      continue;
    }
    checkEq(out[0].type, CRSF_FRAMETYPE_RC_CHANNELS,
            "split at " + std::to_string(split) + ": type");
    checkEq(out[0].payload.size(), 22,
            "split at " + std::to_string(split) + ": payload size");
    checkEq(p.framesCrcFail(), 0,
            "split at " + std::to_string(split) + ": no crc failures");
  }

  // One byte at a time - the pathological case.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    for (size_t i = 0; i < frame.size(); i++)
      p.feed(&frame[i], 1, out);

    checkEq(out.size(), 1, "byte-at-a-time: one frame");
    checkEq(p.bytesDiscarded(), 0, "byte-at-a-time: nothing discarded");
  }

  // Two frames split across three ragged chunks.
  {
    auto a = makeRcFrame(0x11);
    auto b = makeRcFrame(0x22);
    std::vector<uint8_t> all;
    all.insert(all.end(), a.begin(), a.end());
    all.insert(all.end(), b.begin(), b.end());

    CrsfParser p;
    std::vector<CrsfFrame> out;
    size_t c1 = 7, c2 = 31;   // both land mid-frame
    p.feed(all.data(), c1, out);
    p.feed(all.data() + c1, c2 - c1, out);
    p.feed(all.data() + c2, all.size() - c2, out);

    checkEq(out.size(), 2, "ragged chunks: two frames");
    if (out.size() == 2) {
      check(out[0].payload[0] == 0x11, "ragged chunks: first frame payload");
      check(out[1].payload[0] == 0x22, "ragged chunks: second frame payload");
    }
  }
}

//---------------------------------------------------------
// Multiple frames in one chunk

static void testMultipleFramesOneChunk()
{
  std::vector<uint8_t> all;
  for (int i = 0; i < 5; i++) {
    auto f = makeRcFrame((uint8_t)(0x10 + i));
    all.insert(all.end(), f.begin(), f.end());
  }

  CrsfParser p;
  std::vector<CrsfFrame> out;
  size_t n = feedVec(p, all, out);

  checkEq(n, 5, "multi-frame chunk: five emitted");
  checkEq(out.size(), 5, "multi-frame chunk: out size");
  for (size_t i = 0; i < out.size(); i++)
    check(out[i].payload[0] == (uint8_t)(0x10 + i),
          "multi-frame chunk: frame " + std::to_string(i) + " payload");
  checkEq(p.pending(), 0, "multi-frame chunk: buffer drained");
}

//---------------------------------------------------------
// CRC rejection
//
// Every single-bit flip in the payload must be caught.

static void testCrcRejection()
{
  auto good = makeRcFrame();

  // Corrupt the CRC byte itself.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    auto bad = good;
    bad[bad.size() - 1] ^= 0xFF;
    feedVec(p, bad, out);
    checkEq(out.size(), 0, "bad crc: no frame emitted");
    checkEq(p.framesCrcFail(), 1, "bad crc: failure counted");
  }

  // Flip one bit in each payload byte in turn.
  for (size_t byte = 3; byte + 1 < good.size(); byte++) {
    for (int bit = 0; bit < 8; bit++) {
      CrsfParser p;
      std::vector<CrsfFrame> out;
      auto bad = good;
      bad[byte] ^= (uint8_t)(1 << bit);
      feedVec(p, bad, out);

      check(out.empty(),
            "bit flip at byte " + std::to_string(byte) +
            " bit " + std::to_string(bit) + ": frame rejected");
    }
  }
}

//---------------------------------------------------------
// Resync after corruption
//
// The rule under test: on a bad frame, drop ONE byte and rescan.
// Dropping the whole buffer would lose a valid frame that begins
// inside the corrupted bytes and cascade the loss forward.

static void testResync()
{
  // Leading garbage, then a good frame.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    std::vector<uint8_t> stream = {0x01, 0x02, 0x03, 0xFF, 0x7E};
    auto f = makeRcFrame();
    stream.insert(stream.end(), f.begin(), f.end());

    feedVec(p, stream, out);
    checkEq(out.size(), 1, "leading garbage: frame recovered");
    checkEq(p.bytesDiscarded(), 5, "leading garbage: five bytes discarded");
  }

  // Truncated frame followed by a good one. The truncated frame's
  // sync byte claims a length that runs into the next frame, so
  // the CRC fails and only byte-at-a-time resync recovers.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;

    auto truncated = makeRcFrame(0x55);
    truncated.resize(truncated.size() - 4);   // chop the tail

    auto good = makeRcFrame(0x66);

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), truncated.begin(), truncated.end());
    stream.insert(stream.end(), good.begin(), good.end());

    feedVec(p, stream, out);

    checkEq(out.size(), 1, "truncated then good: exactly one frame");
    if (out.size() == 1)
      check(out[0].payload[0] == 0x66,
            "truncated then good: recovered the GOOD frame");
  }

  // A sync byte embedded in payload data must not permanently
  // desync the stream.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;

    std::vector<uint8_t> payload(22, 0x00);
    payload[0] = CRSF_SYNC_BYTE;
    payload[1] = 0x18;
    auto f1 = makeFrame(CRSF_FRAMETYPE_RC_CHANNELS, payload);
    auto f2 = makeRcFrame(0x77);

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());

    feedVec(p, stream, out);
    checkEq(out.size(), 2, "embedded sync byte: both frames decoded");
  }
}

//---------------------------------------------------------
// Malformed length fields
//
// Must be rejected without over-reading the buffer. This is the
// memory-safety case: a len field is attacker-adjacent data in
// the sense that line noise can produce any value.

static void testBadLengths()
{
  // len = 0 and len = 1 are below the minimum (type + crc).
  for (uint8_t bad_len : {(uint8_t)0x00, (uint8_t)0x01}) {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    std::vector<uint8_t> stream = {CRSF_SYNC_BYTE, bad_len, 0x16, 0x00};
    auto f = makeRcFrame();
    stream.insert(stream.end(), f.begin(), f.end());

    feedVec(p, stream, out);
    checkEq(out.size(), 1,
            "len=" + std::to_string((int)bad_len) + ": recovers next frame");
    check(p.framesBadLength() >= 1,
          "len=" + std::to_string((int)bad_len) + ": bad length counted");
  }

  // len beyond the 62-byte maximum.
  for (uint8_t bad_len : {(uint8_t)0x3F, (uint8_t)0x80, (uint8_t)0xFF}) {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    std::vector<uint8_t> stream = {CRSF_SYNC_BYTE, bad_len};
    auto f = makeRcFrame();
    stream.insert(stream.end(), f.begin(), f.end());

    feedVec(p, stream, out);
    checkEq(out.size(), 1,
            "len=" + std::to_string((int)bad_len) + ": recovers next frame");
    check(p.framesBadLength() >= 1,
          "len=" + std::to_string((int)bad_len) + ": bad length counted");
  }

  // Maximum legal frame must be accepted.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    auto f = makeFrame(0x16, std::vector<uint8_t>(CRSF_MAX_PAYLOAD, 0x42));
    checkEq(f.size(), CRSF_MAX_FRAME_SIZE, "max frame is 64 bytes");
    feedVec(p, f, out);
    checkEq(out.size(), 1, "max-size frame accepted");
    if (out.size() == 1)
      checkEq(out[0].payload.size(), CRSF_MAX_PAYLOAD, "max payload size");
  }

  // Minimum legal frame: type + crc, no payload.
  {
    CrsfParser p;
    std::vector<CrsfFrame> out;
    auto f = makeFrame(CRSF_FRAMETYPE_FLIGHT_MODE, {});
    feedVec(p, f, out);
    checkEq(out.size(), 1, "empty-payload frame accepted");
    if (out.size() == 1)
      checkEq(out[0].payload.size(), 0, "empty payload is empty");
  }
}

//---------------------------------------------------------
// Unknown frame types pass through
//
// The parser is type-agnostic; filtering is a higher layer's job.

static void testUnknownTypes()
{
  CrsfParser p;
  std::vector<CrsfFrame> out;

  for (uint8_t type : {(uint8_t)0x00, (uint8_t)0x29, (uint8_t)0x7A, (uint8_t)0xFF}) {
    auto f = makeFrame(type, {0xDE, 0xAD});
    feedVec(p, f, out);
  }

  checkEq(out.size(), 4, "unknown types: all emitted");
  checkEq(p.framesCrcFail(), 0, "unknown types: not treated as errors");
}

//---------------------------------------------------------
// Partial frame stays buffered, no spurious emit

static void testPartialFrameHeld()
{
  CrsfParser p;
  std::vector<CrsfFrame> out;

  auto f = makeRcFrame();
  feedVec(p, std::vector<uint8_t>(f.begin(), f.end() - 1), out);

  checkEq(out.size(), 0, "partial frame: nothing emitted");
  check(p.pending() > 0, "partial frame: bytes held");

  p.feed(&f[f.size() - 1], 1, out);
  checkEq(out.size(), 1, "partial frame: completes on final byte");
  checkEq(p.pending(), 0, "partial frame: buffer drained");
}

//---------------------------------------------------------
// reset() clears parse state but keeps counters

static void testReset()
{
  CrsfParser p;
  std::vector<CrsfFrame> out;

  auto f = makeRcFrame();
  feedVec(p, f, out);
  feedVec(p, std::vector<uint8_t>(f.begin(), f.end() - 3), out);

  check(p.pending() > 0, "reset: partial bytes held before reset");
  p.reset();
  checkEq(p.pending(), 0, "reset: parse state cleared");
  checkEq(p.framesValid(), 1, "reset: counters preserved");

  p.resetStats();
  checkEq(p.framesValid(), 0, "resetStats: counters zeroed");
}

//---------------------------------------------------------
// Sustained stream with periodic corruption
//
// Approximates a marginal link: the parser must keep recovering
// rather than degrading, and must never emit a corrupt frame.

static void testSustainedStreamWithNoise()
{
  CrsfParser p;
  std::vector<CrsfFrame> out;

  const int kFrames = 200;
  int expected_good = 0;

  for (int i = 0; i < kFrames; i++) {
    auto f = makeRcFrame((uint8_t)(i & 0xFF));

    if (i % 10 == 3) {
      f[5] ^= 0x01;               // corrupt payload -> must be rejected
    } else if (i % 10 == 7) {
      std::vector<uint8_t> noise = {0xAA, 0x55, 0xC8, 0x00};
      p.feed(noise.data(), noise.size(), out);
      expected_good++;            // frame itself is still intact
    } else {
      expected_good++;
    }

    feedVec(p, f, out);
  }

  checkEq(out.size(), (uint64_t)expected_good,
          "sustained stream: all good frames recovered");

  // Every emitted frame must be a real RC frame with 22 payload
  // bytes - nothing corrupt should ever reach the caller.
  bool all_well_formed = true;
  for (const auto& fr : out) {
    if (fr.type != CRSF_FRAMETYPE_RC_CHANNELS || fr.payload.size() != 22)
      all_well_formed = false;
  }
  check(all_well_formed, "sustained stream: no malformed frame emitted");
  checkEq(p.framesValid(), (uint64_t)expected_good,
          "sustained stream: valid counter matches");
}

//---------------------------------------------------------
// main()

int main()
{
  printf("CRSF parser unit tests\n");
  printf("======================\n");

  testCrc8();
  testSingleFrame();
  testSyncVariants();
  testChunkBoundaries();
  testMultipleFramesOneChunk();
  testCrcRejection();
  testResync();
  testBadLengths();
  testUnknownTypes();
  testPartialFrameHeld();
  testReset();
  testSustainedStreamWithNoise();

  printf("\n%d checks run\n", g_checks);

  if (g_failures > 0) {
    fprintf(stderr, "\nFAIL: %d of %d checks failed\n", g_failures, g_checks);
    return 1;
  }

  printf("PASS\n");
  return 0;
}
