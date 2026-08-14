/*************************************************************
 * CRSF (Crossfire) frame parser
 *
 * Pure byte-stream framing and validation. NO serial I/O, no
 * platform dependency, no clock - feed() takes bytes from any
 * source and emits complete, CRC-valid frames.
 *
 * This separation is deliberate. lib_sbus calls read() inside
 * update(), which welds parsing to I/O and makes the decoder
 * untestable without a serial port; sbus_probe had to
 * reimplement the parser to get a testable seam, so the repo
 * now carries two SBUS parsers that can disagree. lib_crsf has
 * exactly one parser: the app, the probe, and the unit tests
 * all drive this class.
 *
 * Payload interpretation (channels, link stats, telemetry)
 * belongs in a separate layer. This file only answers "where
 * does a frame start and end, and is it intact?"
 *
 * Wire format:
 *   [sync][len][type][payload...][crc8]
 *   sync - 0xC8, or 0xEE from EdgeTX handsets
 *   len  - counts type + payload + crc (NOT sync or len)
 *   crc8 - poly 0xD5 over type + payload
 *
 * CRSF is 420000 baud 8N1. Unlike SBUS's 8E2 there is no parity
 * requirement, so any Pi UART can frame it - the mini-UART
 * restriction that governs SBUS does not apply here.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#ifndef CRSF_PARSER_H
#define CRSF_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <vector>

// Sync bytes. Receivers emit 0xC8; EdgeTX handsets emit 0xEE on
// their outgoing channel/telemetry packets. Accept both.
#define CRSF_SYNC_BYTE      0xC8
#define CRSF_SYNC_BYTE_EDGE 0xEE

// CRC8 polynomial used across all CRSF frames.
#define CRSF_CRC_POLY       0xD5

// A frame is [sync][len][...len bytes...], so the largest legal
// total is 64 bytes and the largest legal len is 62. Since len
// covers type + payload + crc, payload tops out at 60.
#define CRSF_MAX_FRAME_SIZE 64
#define CRSF_MAX_LEN_FIELD  62
#define CRSF_MAX_PAYLOAD    60

// len must cover at least a type byte and a CRC byte.
#define CRSF_MIN_LEN_FIELD  2

// Frame type IDs. Only the ones this project decodes or emits;
// the parser itself is type-agnostic and passes everything up.
#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21

// One complete, CRC-validated frame. `payload` excludes the type
// byte and the CRC; `type` carries the frame type.
struct CrsfFrame {
  uint8_t type;
  std::vector<uint8_t> payload;
};

// Compute the CRSF CRC8 (poly 0xD5) over a byte range. Exposed
// because both the parser and any frame *encoder* need it.
uint8_t crsfCrc8(const uint8_t* data, size_t len);

class CrsfParser {
public:
  CrsfParser();

  // Feed an arbitrary chunk of bytes. Chunks may split frames at
  // any offset - the parser holds partial state across calls, so
  // callers never need to align reads to frame boundaries.
  //
  // Appends every complete CRC-valid frame found to `out` and
  // returns how many were appended.
  size_t feed(const uint8_t* data, size_t len, std::vector<CrsfFrame>& out);

  // Discard partial-frame state. Counters are left alone.
  void reset();

  // Zero the counters. Does not touch parse state.
  void resetStats();

  // Frames that passed CRC.
  uint64_t framesValid() const { return frames_valid_; }

  // Frames that framed up but failed CRC. A healthy wired link
  // should sit at zero; a nonzero and climbing count means line
  // noise, a baud mismatch, or contention on the port.
  uint64_t framesCrcFail() const { return frames_crc_fail_; }

  // Bytes dropped while hunting for a sync byte. Nonzero after a
  // mid-frame start is normal; continuously climbing is not.
  uint64_t bytesDiscarded() const { return bytes_discarded_; }

  // Frames rejected on an out-of-range len field, before any CRC
  // check. Usually a desync that happened to land on a sync byte.
  uint64_t framesBadLength() const { return frames_bad_length_; }

  // Bytes currently held as an incomplete frame.
  size_t pending() const { return buf_.size(); }

private:
  // Accumulates one candidate frame: buf_[0] is sync, buf_[1] is
  // len, and the frame is complete at 2 + len bytes.
  std::vector<uint8_t> buf_;

  uint64_t frames_valid_;
  uint64_t frames_crc_fail_;
  uint64_t bytes_discarded_;
  uint64_t frames_bad_length_;

  // Drop buf_[0] and rescan the remainder for a sync byte.
  //
  // Resyncing by one byte rather than clearing the buffer matters:
  // a corrupted frame may have a valid frame starting inside it,
  // and dropping the whole buffer would lose it and cascade into
  // further loss. One byte at a time is the only way to guarantee
  // we find the next real frame boundary.
  void resyncOneByte();
};

#endif /* CRSF_PARSER_H */
