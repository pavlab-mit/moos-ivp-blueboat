#include "wire_format.h"

#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace bb {
namespace wire {

SplitResult kv_split(const std::string& text,
                     std::map<std::string, std::string>& out)
{
  SplitResult r;

  size_t pos = 0;
  while (pos <= text.size()) {
    const size_t comma = text.find(',', pos);
    const std::string tok = (comma == std::string::npos)
                                ? text.substr(pos)
                                : text.substr(pos, comma - pos);

    // An empty token is a doubled comma or a trailing one. pos can
    // only reach text.size() here by following a comma, so an
    // empty token at the end is a TRAILING comma, not a normal
    // loop exit -- the loop leaves via the npos break below.
    if (tok.empty()) {
      r.fault = SplitFault::MALFORMED;
      r.detail = "empty token";
      return r;
    }

    const size_t eq = tok.find('=');
    if (eq == std::string::npos || eq == 0) {
      r.fault = SplitFault::MALFORMED;
      r.detail = tok;
      return r;
    }

    const std::string key = tok.substr(0, eq);
    if (out.count(key)) {
      // Last-writer-wins on a duplicated surge is exactly the
      // ambiguity this contract exists to remove.
      r.fault = SplitFault::DUPLICATE_KEY;
      r.detail = key;
      return r;
    }
    out[key] = tok.substr(eq + 1);

    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return r;
}

bool parse_double(const std::string& s, double& out)
{
  if (s.empty()) return false;
  const char* begin = s.c_str();
  char* end = nullptr;
  const double v = std::strtod(begin, &end);
  if (end == begin) return false;
  while (*end == ' ') ++end;          // trailing spaces tolerated
  if (*end != '\0') return false;     // any other garbage rejected
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

std::string formatf(const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  va_list args2;
  va_copy(args2, args);

  const int needed = std::vsnprintf(nullptr, 0, fmt, args);
  va_end(args);
  if (needed < 0) { va_end(args2); return std::string(); }

  std::vector<char> buf((size_t)needed + 1);
  std::vsnprintf(buf.data(), buf.size(), fmt, args2);
  va_end(args2);
  return std::string(buf.data(), (size_t)needed);
}

} // namespace wire

bool parse_bool_token(const std::string& s, bool fallback)
{
  std::string t;
  t.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    t += (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
  }
  if (t == "true"  || t == "1") return true;
  if (t == "false" || t == "0") return false;
  return fallback;
}

} // namespace bb
