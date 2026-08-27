/*************************************************************
 *  wire_format -- the ONE strict key=value wire dialect.
 *
 *  Shared by the three contract parsers (command envelope,
 *  BB_SELECTED_CMD, BB_MIXED_CMD). It exists because the parsers
 *  started as three private copies and had ALREADY diverged by
 *  the time of the pre-hardware audit: the envelope's number
 *  parser tolerated trailing spaces while the other two rejected
 *  them, and only the envelope checked u64 overflow. One dialect,
 *  defined once, pinned by the tests of all three parsers.
 *
 *  Semantics (the union of what the three parsers required):
 *    - tokens split on ',', every token exactly key=value with a
 *      non-empty key; doubled/trailing commas and bare tokens are
 *      malformed; duplicate keys are their own fault class;
 *    - doubles: strict strtod, trailing spaces tolerated, any
 *      other trailing garbage rejected, non-finite rejected;
 *    - u64: decimal digits only, ERANGE rejected.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_WIRE_FORMAT_HEADER
#define BB_WIRE_FORMAT_HEADER

#include <cstdint>
#include <map>
#include <string>

namespace bb {
namespace wire {

enum class SplitFault { NONE, MALFORMED, DUPLICATE_KEY };

struct SplitResult
{
  SplitFault  fault = SplitFault::NONE;
  std::string detail;   // offending token or key
};

// Split one wire string into key/value pairs. Fails closed on the
// first malformed token or duplicate key; `out` may hold a partial
// map afterwards and must be discarded by the caller on fault.
SplitResult kv_split(const std::string& text,
                     std::map<std::string, std::string>& out);

// Strict numeric parses, per the dialect above. Return false on
// any rejection; `out` untouched on failure.
bool parse_double(const std::string& s, double& out);
bool parse_u64(const std::string& s, uint64_t& out);

// printf into a std::string with correct sizing -- no fixed
// buffer, so a long epoch or producer name can never be silently
// truncated into a frame that fails parse downstream.
std::string formatf(const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

} // namespace wire

// Relaxed boolean for the plain-scalar SIDEBANDS and policy flags
// (RC_KILL_ASSERTED, RC_LINK_ALIVE, NVGR_DISARM, ALL_STOP ...),
// which are deliberately not contract envelopes. Both of a
// sideband's consumers must read a value the same way -- the
// pre-audit code accepted "1" at the arbiter but not at the
// Navigator, i.e. the ENFORCING side was the stricter parser.
// Case-insensitive "true"/"1" -> true, "false"/"0" -> false,
// anything else -> fallback (never a silent flip).
bool parse_bool_token(const std::string& s, bool fallback);

} // namespace bb

#endif
