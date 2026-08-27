#include "command_envelope.h"
#include "wire_format.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <ctime>

namespace bb {

namespace reject {
const char* kMalformed           = "malformed";
const char* kDuplicateKey        = "duplicate_key";
const char* kUnsupportedVersion  = "unsupported_version";
const char* kNonFinite           = "nonfinite";
const char* kOutOfRange          = "out_of_range";
const char* kInvalidFlag         = "invalid_flag";
const char* kMissingField        = "missing_field";
}

std::string make_epoch(const std::string& prefix)
{
  // The counter is what makes two epochs minted by ONE process in
  // ONE second distinct -- pid ^ time alone reseeded to the same
  // value and collided. No caller does that today; this is the
  // guard for the first one that does.
  static std::atomic<unsigned> s_mint_count{0};
  unsigned seed = (unsigned)::getpid();
  seed ^= (unsigned)::time(NULL) * 2654435761u;
  seed ^= (s_mint_count.fetch_add(1) + 1u) * 0x9E3779B9u;
  ::srandom(seed);
  return wire::formatf("%s-%06lx", prefix.c_str(),
                       (long)(::random() & 0xFFFFFF));
}

const char* to_string(CommandSource s)
{
  switch (s) {
    case CommandSource::RC:            return "RC";
    case CommandSource::TELEOP:        return "TELEOP";
    case CommandSource::AUTONOMY:      return "AUTONOMY";
    case CommandSource::LEGACY_DIRECT: return "LEGACY_DIRECT";
    case CommandSource::NONE:          break;
  }
  return "NONE";
}

std::string SemanticCommand::field(const std::string& key,
                                   const std::string& fallback) const
{
  std::map<std::string, std::string>::const_iterator it = extra.find(key);
  return (it == extra.end()) ? fallback : it->second;
}

bool SemanticCommand::field_bool(const std::string& key, bool fallback) const
{
  std::map<std::string, std::string>::const_iterator it = extra.find(key);
  if (it == extra.end()) return fallback;
  if (it->second == "1") return true;
  if (it->second == "0") return false;
  return fallback;
}

namespace {

ParseResult fail(const char* reason, const std::string& detail)
{
  ParseResult r;
  r.ok = false;
  r.reject_reason = reason;
  r.detail = detail;
  return r;
}

// Numeric parses come from wire_format -- the ONE dialect all
// three contract parsers share.
using wire::parse_double;
using wire::parse_u64;

// Flags are strictly "0" or "1". Accepting "true"/"yes"/"" is how
// a typo silently becomes a permissive default.
bool parse_flag(const std::string& s, bool& out)
{
  if (s == "1") { out = true;  return true; }
  if (s == "0") { out = false; return true; }
  return false;
}

} // namespace

ParseResult parse_command(const std::string& text, CommandSource source)
{
  if (text.empty())
    return fail(reject::kMalformed, "empty");

  std::map<std::string, std::string> kv;
  const wire::SplitResult sr = wire::kv_split(text, kv);
  if (sr.fault == wire::SplitFault::DUPLICATE_KEY)
    return fail(reject::kDuplicateKey, sr.detail);
  if (sr.fault != wire::SplitFault::NONE)
    return fail(reject::kMalformed, sr.detail);

  // --- version first: reject an unsupported contract before
  //     interpreting any field under v1 assumptions.
  std::map<std::string, std::string>::iterator it = kv.find("v");
  if (it == kv.end())
    return fail(reject::kMissingField, "v");
  uint64_t version_u = 0;
  if (!parse_u64(it->second, version_u))
    return fail(reject::kMalformed, "v");
  if (version_u != kCommandContractVersion)
    return fail(reject::kUnsupportedVersion, it->second);

  SemanticCommand cmd;
  cmd.version = (uint32_t)version_u;
  cmd.source  = source;

  // --- required string fields
  const char* required_strings[] = {"producer", "epoch"};
  for (size_t i = 0; i < 2; ++i) {
    it = kv.find(required_strings[i]);
    if (it == kv.end() || it->second.empty())
      return fail(reject::kMissingField, required_strings[i]);
  }
  cmd.producer = kv["producer"];
  cmd.epoch    = kv["epoch"];

  // --- seq
  it = kv.find("seq");
  if (it == kv.end())
    return fail(reject::kMissingField, "seq");
  if (!parse_u64(it->second, cmd.seq))
    return fail(reject::kMalformed, "seq");

  // --- source_time (diagnostic, but must still be a number)
  it = kv.find("source_time");
  if (it == kv.end())
    return fail(reject::kMissingField, "source_time");
  if (!parse_double(it->second, cmd.source_time))
    return fail(reject::kNonFinite, "source_time");

  // --- valid flag
  it = kv.find("valid");
  if (it == kv.end())
    return fail(reject::kMissingField, "valid");
  if (!parse_flag(it->second, cmd.valid))
    return fail(reject::kInvalidFlag, "valid");

  // --- surge / yaw
  const char* axes[] = {"surge", "yaw"};
  double axis_vals[2] = {0.0, 0.0};
  for (size_t i = 0; i < 2; ++i) {
    it = kv.find(axes[i]);
    if (it == kv.end())
      return fail(reject::kMissingField, axes[i]);
    if (!parse_double(it->second, axis_vals[i]))
      return fail(reject::kNonFinite, axes[i]);
    if (axis_vals[i] < -100.0 || axis_vals[i] > 100.0)
      return fail(reject::kOutOfRange, axes[i]);
  }
  cmd.surge = axis_vals[0];
  cmd.yaw   = axis_vals[1];

  // --- authority_limit is optional; absent means full authority.
  it = kv.find("authority_limit");
  if (it != kv.end()) {
    if (!parse_double(it->second, cmd.authority_limit))
      return fail(reject::kNonFinite, "authority_limit");
    if (cmd.authority_limit < 0.0 || cmd.authority_limit > 100.0)
      return fail(reject::kOutOfRange, "authority_limit");
  }

  // --- everything else is source-specific or unknown-optional.
  static const char* consumed[] = {
      "v", "producer", "epoch", "seq", "source_time",
      "valid", "surge", "yaw", "authority_limit"};
  for (std::map<std::string, std::string>::const_iterator k = kv.begin();
       k != kv.end(); ++k) {
    bool is_consumed = false;
    for (size_t i = 0; i < sizeof(consumed) / sizeof(consumed[0]); ++i) {
      if (k->first == consumed[i]) { is_consumed = true; break; }
    }
    if (!is_consumed) cmd.extra[k->first] = k->second;
  }

  ParseResult r;
  r.ok = true;
  r.command = cmd;
  return r;
}

std::string serialize_command(const SemanticCommand& cmd)
{
  std::string out = wire::formatf(
      "v=%u,producer=%s,epoch=%s,seq=%llu,source_time=%.3f,"
      "valid=%d,surge=%.3f,yaw=%.3f,authority_limit=%.3f",
      (unsigned)cmd.version,
      cmd.producer.c_str(),
      cmd.epoch.c_str(),
      (unsigned long long)cmd.seq,
      cmd.source_time,
      cmd.valid ? 1 : 0,
      cmd.surge,
      cmd.yaw,
      cmd.authority_limit);
  for (std::map<std::string, std::string>::const_iterator k = cmd.extra.begin();
       k != cmd.extra.end(); ++k) {
    out += ",";
    out += k->first;
    out += "=";
    out += k->second;
  }
  return out;
}

} // namespace bb
