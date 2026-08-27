/*************************************************************
 * Unit tests for the shared wire dialect (wire_format.h).
 *
 * These pin the ONE set of parsing semantics all three contract
 * parsers share. They exist because the parsers started as three
 * private copies and had already diverged by the pre-hardware
 * audit -- trailing-space tolerance and u64 overflow checking
 * differed between them. The cross-parser checks at the bottom
 * are the point: parse_decision and parse_mixed must accept and
 * reject exactly what parse_command does, per field type.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "wire_format.h"
#include "authority.h"
#include "mixer_stage.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) { g_failures++; fprintf(stderr, "FAIL: %s\n", what.c_str()); }
}

// ---------------------------------------------------------

static void test_kv_split()
{
  std::map<std::string, std::string> kv;

  wire::SplitResult r = wire::kv_split("a=1,b=2,c=", kv);
  check(r.fault == wire::SplitFault::NONE, "clean split accepted");
  check(kv.size() == 3 && kv["a"] == "1" && kv["c"] == "",
        "clean split values (empty value legal)");

  kv.clear();
  r = wire::kv_split("a=1,,b=2", kv);
  check(r.fault == wire::SplitFault::MALFORMED, "doubled comma malformed");

  kv.clear();
  r = wire::kv_split("a=1,b=2,", kv);
  check(r.fault == wire::SplitFault::MALFORMED, "trailing comma malformed");

  kv.clear();
  r = wire::kv_split("a=1,=2", kv);
  check(r.fault == wire::SplitFault::MALFORMED, "empty key malformed");

  kv.clear();
  r = wire::kv_split("a=1,bare", kv);
  check(r.fault == wire::SplitFault::MALFORMED, "bare token malformed");

  kv.clear();
  r = wire::kv_split("a=1,a=2", kv);
  check(r.fault == wire::SplitFault::DUPLICATE_KEY && r.detail == "a",
        "duplicate key labelled as such");

  // The audit's labelling edge: a MALFORMED token whose text
  // happens to match an existing key must stay malformed. The old
  // kv.count(bad) inference in the decision/mixed parsers called
  // this a duplicate.
  kv.clear();
  r = wire::kv_split("a=1,a", kv);
  check(r.fault == wire::SplitFault::MALFORMED && r.detail == "a",
        "bare token matching an existing key is malformed, not duplicate");
}

static void test_numbers()
{
  double d = 0.0;
  check(wire::parse_double("42.5", d) && d == 42.5, "plain double");
  check(wire::parse_double("42.5 ", d),            "trailing space tolerated");
  check(!wire::parse_double("42.5x", d),           "trailing garbage rejected");
  check(!wire::parse_double(" 42.5", d) || d == 42.5,
        "leading space follows strtod semantics");
  check(!wire::parse_double("nan", d),  "nan rejected");
  check(!wire::parse_double("inf", d),  "inf rejected");
  check(!wire::parse_double("", d),     "empty rejected");

  uint64_t u = 0;
  check(wire::parse_u64("18446744073709551615", u) &&
        u == 18446744073709551615ull, "u64 max accepted");
  check(!wire::parse_u64("18446744073709551616", u), "u64 overflow rejected");
  check(!wire::parse_u64("-1", u),  "negative rejected");
  check(!wire::parse_u64("1.0", u), "decimal point rejected");
  check(!wire::parse_u64("", u),    "empty rejected");
}

static void test_formatf()
{
  check(wire::formatf("x=%d", 7) == "x=7", "formatf basic");

  // The whole reason formatf exists: no fixed buffer, so a long
  // field cannot be silently truncated into a frame that fails
  // parse downstream.
  const std::string long_epoch(2000, 'e');
  const std::string out = wire::formatf("epoch=%s,seq=%d", long_epoch.c_str(), 3);
  check(out.size() == 6 + 2000 + 6, "formatf sizes past any fixed buffer");
  check(out.substr(out.size() - 6) == ",seq=3", "formatf tail intact");
}

static void test_bool_token()
{
  check(parse_bool_token("true", false)  == true,  "true");
  check(parse_bool_token("TRUE", false)  == true,  "TRUE");
  check(parse_bool_token("1", false)     == true,  "\"1\"");
  check(parse_bool_token("false", true)  == false, "false");
  check(parse_bool_token("0", true)      == false, "\"0\"");
  // Anything else keeps the caller's current value -- a typo must
  // never silently flip a safety sideband either way.
  check(parse_bool_token("yes", false)   == false, "junk keeps fallback (false)");
  check(parse_bool_token("yes", true)    == true,  "junk keeps fallback (true)");
  check(parse_bool_token("", true)       == true,  "empty keeps fallback");
}

static void test_make_epoch_distinct()
{
  // Two epochs minted by one process in one second must differ --
  // the counter mixed into the seed is what guarantees it.
  std::set<std::string> seen;
  for (int i = 0; i < 8; ++i)
    seen.insert(make_epoch("t"));
  check(seen.size() == 8, "8 epochs minted back-to-back are distinct");
}

// ---------------------------------------------------------
// Cross-parser: the decision and mixed parsers speak the SAME
// dialect as the envelope. Trailing-space tolerance is the case
// that had actually diverged.

static void test_cross_parser_semantics()
{
  DecisionParseResult dr = parse_decision(
      "v=1,decision_epoch=arb-1,decision_seq=9,decision_time=1.0,"
      "selected=RC,hard_stop=0,stop=NONE,surge=42.5 ,yaw=-8.0");
  check(dr.ok, "parse_decision tolerates trailing space in a number");

  MixedParseResult mr = parse_mixed(
      "v=1,mixer_model=M,mix_epoch=mix-1,mix_seq=3,mix_time=1.0,"
      "selected=RC,hard_stop=0,stop=NONE,left_effort=33.8 ,right_effort=49.8");
  check(mr.ok, "parse_mixed tolerates trailing space in a number");

  dr = parse_decision(
      "v=1,decision_epoch=arb-1,decision_seq=99999999999999999999,"
      "decision_time=1.0,selected=RC,hard_stop=0,stop=NONE,surge=0,yaw=0");
  check(!dr.ok, "parse_decision rejects u64 overflow");

  dr = parse_decision("v=1,decision_epoch=arb-1,decision_seq=9,seq");
  check(!dr.ok && dr.reject_reason == reject::kMalformed,
        "parse_decision: bare token is malformed even if it matches a key-ish name");
}

int main()
{
  test_kv_split();
  test_numbers();
  test_formatf();
  test_bool_token();
  test_make_epoch_distinct();
  test_cross_parser_semantics();

  fprintf(stderr, "%s: %d checks, %d failures\n",
          g_failures ? "FAILED" : "OK", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
