/*************************************************************
 * CRSF frame parser implementation
 *
 * See crsf_parser.h for the wire format and design rationale.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "crsf_parser.h"

//---------------------------------------------------------
// crsfCrc8()
//
// CRSF uses a bitwise CRC8 with polynomial 0xD5, no reflection,
// zero init. Covers type + payload, i.e. buf[2 .. 2+len-2).
//
// Kept as a straightforward bitwise loop rather than a lookup
// table: frames are <= 64 bytes at 420 kbaud, so the table would
// buy nothing measurable and would be one more thing to get
// wrong.

uint8_t crsfCrc8(const uint8_t* data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ CRSF_CRC_POLY);
      else
        crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

//---------------------------------------------------------
// Constructor

CrsfParser::CrsfParser()
{
  resetStats();
  buf_.reserve(CRSF_MAX_FRAME_SIZE);
}

//---------------------------------------------------------
// reset() / resetStats()

void CrsfParser::reset()
{
  buf_.clear();
}

void CrsfParser::resetStats()
{
  frames_valid_      = 0;
  frames_crc_fail_   = 0;
  bytes_discarded_   = 0;
  frames_bad_length_ = 0;
}

//---------------------------------------------------------
// resyncOneByte()
//
// Drop the leading byte, then skip ahead to the next plausible
// sync byte. Every byte skipped counts as discarded.
//
// One byte at a time, never a wholesale buffer clear: a valid
// frame can begin inside the bytes of a corrupted one, and
// clearing would drop it and cascade the loss forward.

void CrsfParser::resyncOneByte()
{
  if (buf_.empty())
    return;

  // Drop the byte that failed to start a valid frame.
  buf_.erase(buf_.begin());
  bytes_discarded_++;

  // Advance to the next candidate sync byte, counting what we skip.
  size_t skip = 0;
  while (skip < buf_.size() &&
         buf_[skip] != CRSF_SYNC_BYTE &&
         buf_[skip] != CRSF_SYNC_BYTE_EDGE) {
    skip++;
  }

  if (skip > 0) {
    buf_.erase(buf_.begin(), buf_.begin() + skip);
    bytes_discarded_ += skip;
  }
}

//---------------------------------------------------------
// feed()
//
// Append the chunk, then consume as many complete frames as the
// buffer holds. Partial frames stay buffered for the next call,
// which is what makes the parser independent of how the caller
// happens to chunk its reads.

size_t CrsfParser::feed(const uint8_t* data, size_t len,
                        std::vector<CrsfFrame>& out)
{
  if (data != nullptr && len > 0)
    buf_.insert(buf_.end(), data, data + len);

  size_t emitted = 0;

  while (!buf_.empty()) {

    // 1. buf_[0] must be a sync byte. If not, hunt for one.
    if (buf_[0] != CRSF_SYNC_BYTE && buf_[0] != CRSF_SYNC_BYTE_EDGE) {
      resyncOneByte();
      continue;
    }

    // 2. Need the len byte before we know how long the frame is.
    if (buf_.size() < 2)
      break;

    const uint8_t len_field = buf_[1];

    // 3. Validate len before trusting it. An out-of-range len
    //    means this sync byte was data, not a real frame start -
    //    resync rather than waiting forever for bytes that will
    //    never constitute a valid frame.
    if (len_field < CRSF_MIN_LEN_FIELD || len_field > CRSF_MAX_LEN_FIELD) {
      frames_bad_length_++;
      resyncOneByte();
      continue;
    }

    // 4. Whole frame present? total = sync + len + len_field bytes.
    const size_t total = 2 + (size_t)len_field;
    if (buf_.size() < total)
      break;   // incomplete - wait for more bytes

    // 5. CRC covers type + payload: buf_[2 .. total-1), excluding
    //    the trailing CRC byte itself.
    const size_t crc_span = (size_t)len_field - 1;
    const uint8_t crc_calc = crsfCrc8(&buf_[2], crc_span);
    const uint8_t crc_recv = buf_[total - 1];

    if (crc_calc != crc_recv) {
      // Corrupt, or a sync byte that was really payload data.
      // Resync by one byte so a frame starting inside this one
      // is still recoverable.
      frames_crc_fail_++;
      resyncOneByte();
      continue;
    }

    // 6. Good frame. Payload excludes type and CRC.
    CrsfFrame frame;
    frame.type = buf_[2];
    if (crc_span > 1)
      frame.payload.assign(buf_.begin() + 3, buf_.begin() + (total - 1));

    out.push_back(frame);
    frames_valid_++;
    emitted++;

    buf_.erase(buf_.begin(), buf_.begin() + total);
  }

  return emitted;
}
