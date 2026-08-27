/*************************************************************
 *  command_envelope -- the wire contract every command source
 *  publishes, and its strict parser.
 *
 *  One command is ONE MOOS string. Surge, yaw, validity and
 *  sequence travel together or not at all (invariant 5). The old
 *  pipeline combined separate scalar mail into a command pair,
 *  which means there was always a window where half of a command
 *  was new and half was stale.
 *
 *  Format (docs/ibb_navigator_command_pipeline.md §4):
 *    v=1,producer=iRCInterface,epoch=7f2c91,seq=18422,
 *    source_time=381.420,valid=1,surge=42.5,yaw=-8.0,...
 *
 *  The parser is deliberately strict and fails CLOSED. A field it
 *  does not understand in a REQUIRED position, a duplicate key, a
 *  NaN, an out-of-range value, or an unknown major version all
 *  invalidate the entire snapshot rather than yielding a
 *  partially-trusted command (invariant 12).
 *
 *  Unknown OPTIONAL keys are preserved in `extra` and ignored, so
 *  a v1 consumer keeps working against a producer that has learnt
 *  new optional fields.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_COMMAND_ENVELOPE_HEADER
#define BB_COMMAND_ENVELOPE_HEADER

#include <cstdint>
#include <map>
#include <string>

namespace bb {

// Assigned by the receiving mailbox from which variable the mail
// arrived on. Never trusted from the text itself -- a source must
// not be able to claim to be another source (invariant 4).
enum class CommandSource { NONE, RC, TELEOP, AUTONOMY, LEGACY_DIRECT };

const char* to_string(CommandSource s);

static const uint32_t kCommandContractVersion = 1;

struct SemanticCommand
{
  uint32_t      version      = 0;
  CommandSource source       = CommandSource::NONE;
  std::string   producer;
  std::string   epoch;
  uint64_t      seq          = 0;
  double        source_time  = 0.0;   // diagnostic only; never expiry
  bool          valid        = false;
  double        surge        = 0.0;   // [-100, 100], + forward
  double        yaw          = 0.0;   // [-100, 100], + starboard
  double        authority_limit = 100.0;  // [0, 100]

  // Local arrival time of the sample that was accepted. This, not
  // source_time, drives freshness -- front and back seat clocks
  // are not synchronised (invariant 8).
  double        rx_time      = 0.0;

  // Source-specific and unknown-optional fields, verbatim.
  std::map<std::string, std::string> extra;

  // Convenience accessors for source-specific fields. Return the
  // fallback when the key is absent or unparseable.
  std::string field(const std::string& key, const std::string& fallback = "") const;
  bool        field_bool(const std::string& key, bool fallback) const;
};

// Stable reject tokens (docs §12). These go into logs and tests;
// human-readable detail belongs in appcasts.
namespace reject {
extern const char* kMalformed;
extern const char* kDuplicateKey;
extern const char* kUnsupportedVersion;
extern const char* kNonFinite;
extern const char* kOutOfRange;
extern const char* kInvalidFlag;
extern const char* kMissingField;
}

struct ParseResult
{
  bool            ok = false;
  std::string     reject_reason;   // one of reject::*, empty when ok
  std::string     detail;          // which key, for the appcast
  SemanticCommand command;
};

// Mint a per-launch epoch identifier.
//
// Deliberately NOT wall-clock (design doc section 4): two boats
// booting in the same second must not collide, and a clock that
// steps backwards must not make a new session look like an old
// one. Random plus pid is enough for a log join key.
std::string make_epoch(const std::string& prefix);

// Parse one wire string. `source` is stamped onto the result by
// the caller's mailbox; it is not read from the text.
ParseResult parse_command(const std::string& text, CommandSource source);

// Serialise. Round-trips through parse_command for any command
// this library produced.
std::string serialize_command(const SemanticCommand& cmd);

} // namespace bb

#endif
