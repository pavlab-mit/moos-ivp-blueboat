/*************************************************************
 * Unit tests for the command envelope parser.
 *
 * Every test here is a fail-closed test. The parser's job is to
 * refuse anything it does not fully understand, because the
 * alternative -- a partially-trusted command reaching the
 * actuators -- is the failure mode the whole contract exists to
 * prevent (invariant 12).
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "command_envelope.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) { g_failures++; fprintf(stderr, "FAIL: %s\n", what.c_str()); }
}

static const char* kGood =
    "v=1,producer=iRCInterface,epoch=7f2c91,seq=18422,source_time=381.420,"
    "valid=1,surge=42.5,yaw=-8.0,mode=MANUAL,kill=0,authority_limit=65";

static void expect_reject(const std::string& text, const char* reason,
                          const std::string& what)
{
  ParseResult r = parse_command(text, CommandSource::RC);
  if (r.ok) {
    g_checks++; g_failures++;
    fprintf(stderr, "FAIL: %s (accepted, should have rejected)\n", what.c_str());
    return;
  }
  check(r.reject_reason == reason,
        what + " -> reason '" + r.reject_reason + "' expected '" + reason + "'");
}

static void test_happy_path()
{
  ParseResult r = parse_command(kGood, CommandSource::RC);
  check(r.ok, "canonical example parses");
  if (!r.ok) return;

  const SemanticCommand& c = r.command;
  check(c.version == 1,                     "version");
  check(c.producer == "iRCInterface",       "producer");
  check(c.epoch == "7f2c91",                "epoch");
  check(c.seq == 18422ull,                  "seq");
  check(std::fabs(c.source_time - 381.420) < 1e-9, "source_time");
  check(c.valid,                            "valid flag");
  check(std::fabs(c.surge - 42.5) < 1e-9,   "surge");
  check(std::fabs(c.yaw + 8.0) < 1e-9,      "yaw");
  check(std::fabs(c.authority_limit - 65.0) < 1e-9, "authority_limit");

  // Source is stamped by the mailbox, never read from the text.
  check(c.source == CommandSource::RC, "source stamped by caller");

  // Source-specific fields survive in extra.
  check(c.field("mode") == "MANUAL", "extra: mode");
  check(c.field("kill") == "0",      "extra: kill");
  check(c.field_bool("kill", true) == false, "extra: kill as bool");
  check(c.field("absent", "dflt") == "dflt", "extra: fallback");
}

static void test_source_is_not_trusted_from_text()
{
  // Invariant 4: a source must not be able to claim to be another.
  // Even if the text says producer=iTeleop, the mailbox's own
  // channel decides what this is.
  ParseResult r = parse_command(
      "v=1,producer=iTeleop,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
      CommandSource::AUTONOMY);
  check(r.ok, "parses");
  check(r.command.source == CommandSource::AUTONOMY,
        "caller's channel wins over the producer string");
}

static void test_missing_required_fields()
{
  const char* required[] = {"v", "producer", "epoch", "seq",
                            "source_time", "valid", "surge", "yaw"};
  for (size_t i = 0; i < 8; ++i) {
    // Rebuild the canonical string without one required key.
    std::string out;
    std::string src(kGood);
    size_t pos = 0;
    while (pos <= src.size()) {
      size_t comma = src.find(',', pos);
      std::string tok = (comma == std::string::npos)
                            ? src.substr(pos) : src.substr(pos, comma - pos);
      const std::string key = tok.substr(0, tok.find('='));
      if (key != required[i]) {
        if (!out.empty()) out += ",";
        out += tok;
      }
      if (comma == std::string::npos) break;
      pos = comma + 1;
    }
    // Dropping "v" is reported as a missing field too.
    expect_reject(out, reject::kMissingField,
                  std::string("missing ") + required[i]);
  }
}

static void test_duplicate_key()
{
  expect_reject(
      "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=10,yaw=0,surge=90",
      reject::kDuplicateKey, "duplicate surge");
  // Duplicated non-required keys are rejected too -- ambiguity is
  // ambiguity wherever it appears.
  expect_reject(
      "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=1,yaw=0,mode=A,mode=B",
      reject::kDuplicateKey, "duplicate optional key");
}

static void test_version()
{
  expect_reject(
      "v=2,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
      reject::kUnsupportedVersion, "future major version");
  expect_reject(
      "v=x,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
      reject::kMalformed, "non-numeric version");
}

static void test_nonfinite_and_range()
{
  const char* bad_nums[] = {"nan", "NaN", "inf", "-inf", "1e999", "", "12abc", "--5"};
  for (size_t i = 0; i < 8; ++i) {
    std::string t = std::string(
        "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,yaw=0,surge=") + bad_nums[i];
    ParseResult r = parse_command(t, CommandSource::RC);
    check(!r.ok, std::string("surge='") + bad_nums[i] + "' rejected");
  }

  expect_reject("v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=101,yaw=0",
                reject::kOutOfRange, "surge above 100");
  expect_reject("v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=-100.5",
                reject::kOutOfRange, "yaw below -100");
  expect_reject("v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0,"
                "authority_limit=120",
                reject::kOutOfRange, "authority_limit above 100");

  // Exact boundaries are legal.
  ParseResult ok = parse_command(
      "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=100,yaw=-100,"
      "authority_limit=0", CommandSource::RC);
  check(ok.ok, "boundary values accepted");
}

static void test_flags_are_strict()
{
  const char* bad_flags[] = {"true", "TRUE", "yes", "2", "", "-1"};
  for (size_t i = 0; i < 6; ++i) {
    std::string t = std::string(
        "v=1,producer=p,epoch=a,seq=1,source_time=0,surge=0,yaw=0,valid=") + bad_flags[i];
    expect_reject(t, reject::kInvalidFlag,
                  std::string("valid='") + bad_flags[i] + "'");
  }
}

static void test_malformed_structure()
{
  expect_reject("", reject::kMalformed, "empty string");
  expect_reject("no-equals-anywhere", reject::kMalformed, "token without '='");
  expect_reject("=1,producer=p", reject::kMalformed, "empty key");
  expect_reject("v=1,,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
                reject::kMalformed, "doubled comma");
  expect_reject("v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0,",
                reject::kMalformed, "trailing comma");
}

static void test_authority_limit_defaults_to_full()
{
  ParseResult r = parse_command(
      "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
      CommandSource::AUTONOMY);
  check(r.ok, "parses without authority_limit");
  check(std::fabs(r.command.authority_limit - 100.0) < 1e-9,
        "absent authority_limit means full authority");
}

static void test_unknown_optional_keys_are_forward_compatible()
{
  ParseResult r = parse_command(
      "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0,"
      "some_future_field=42,another=xyz", CommandSource::RC);
  check(r.ok, "unknown optional keys do not break a v1 consumer");
  check(r.command.field("some_future_field") == "42", "preserved in extra");
}

static void test_round_trip()
{
  ParseResult a = parse_command(kGood, CommandSource::RC);
  check(a.ok, "round trip: initial parse");
  const std::string wire = serialize_command(a.command);
  ParseResult b = parse_command(wire, CommandSource::RC);
  check(b.ok, "round trip: reparse");
  if (!a.ok || !b.ok) return;

  check(b.command.producer == a.command.producer, "round trip producer");
  check(b.command.epoch    == a.command.epoch,    "round trip epoch");
  check(b.command.seq      == a.command.seq,      "round trip seq");
  check(b.command.valid    == a.command.valid,    "round trip valid");
  check(std::fabs(b.command.surge - a.command.surge) < 1e-3, "round trip surge");
  check(std::fabs(b.command.yaw   - a.command.yaw)   < 1e-3, "round trip yaw");
  check(b.command.field("mode") == "MANUAL", "round trip preserves extra");
}

int main()
{
  printf("test_command_envelope\n");
  test_happy_path();
  test_source_is_not_trusted_from_text();
  test_missing_required_fields();
  test_duplicate_key();
  test_version();
  test_nonfinite_and_range();
  test_flags_are_strict();
  test_malformed_structure();
  test_authority_limit_defaults_to_full();
  test_unknown_optional_keys_are_forward_compatible();
  test_round_trip();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
