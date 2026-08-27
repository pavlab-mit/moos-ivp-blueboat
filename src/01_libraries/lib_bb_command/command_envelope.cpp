#include "command_envelope.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>

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

// Strict numeric parse. Rejects trailing garbage, empty strings,
// and anything non-finite. strtod alone would happily accept
// "42abc" and "nan".
bool parse_double(const std::string& s, double& out)
{
  if (s.empty()) return false;
  const char* begin = s.c_str();
  char* end = nullptr;
  const double v = std::strtod(begin, &end);
  if (end == begin) return false;
  while (*end == ' ') ++end;
  if (*end != '\0') return false;
  if (!std::isfinite(v)) return false;
  out = v;
  return true;
}

bool parse_u64(const std::string& s, uint64_t& out)
{
  if (s.empty()) return false;
  for (size_t i = 0; i < s.size(); ++i)
    if (s[i] < '0' || s[i] > '9') return false;
  errno = 0;
  const unsigned long long v = std::strtoull(s.c_str(), nullptr, 10);
  if (errno == ERANGE) return false;
  out = (uint64_t)v;
  return true;
}

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
  std::set<std::string> seen;

  size_t pos = 0;
  while (pos <= text.size()) {
    const size_t comma = text.find(',', pos);
    const std::string tok = (comma == std::string::npos)
                                ? text.substr(pos)
                                : text.substr(pos, comma - pos);

    // An empty token means a doubled comma or a trailing one.
    // Skipping it would be the permissive reading; this parser
    // does not do permissive readings.
    // pos can only reach text.size() here by following a comma,
    // so an empty token at the end is a TRAILING comma, not a
    // normal loop exit -- the loop leaves via the npos break
    // below. Both are malformed.
    if (tok.empty())
      return fail(reject::kMalformed, "empty token");

    {
      const size_t eq = tok.find('=');
      if (eq == std::string::npos || eq == 0)
        return fail(reject::kMalformed, tok);
      const std::string key = tok.substr(0, eq);
      const std::string val = tok.substr(eq + 1);

      // Duplicate keys are rejected outright. Last-writer-wins on
      // a duplicated surge is exactly the ambiguity this contract
      // exists to remove.
      if (!seen.insert(key).second)
        return fail(reject::kDuplicateKey, key);

      kv[key] = val;
    }

    if (comma == std::string::npos) break;
    pos = comma + 1;
  }

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
  char buf[512];
  std::snprintf(buf, sizeof(buf),
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

  std::string out(buf);
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
