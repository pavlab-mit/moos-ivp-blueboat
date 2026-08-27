/*************************************************************
 *  scenarios -- synthetic command streams for the cases no
 *  recorded log contains.
 *
 *  Real logs are the happy path. Every fail-closed transition in
 *  the design doc's section 10 has to be driven by CONSTRUCTED
 *  input: a source that goes stale mid-command, a producer that
 *  restarts, a broker that repeats one sequence forever, a kill
 *  asserted while driving, a mixer that stops publishing.
 *
 *  These streams are the safety argument for going
 *  unit -> replay -> dock -> water with no on-water shadow phase.
 *  Without them, "replay is the gate" gates only the behaviour
 *  that already worked.
 *
 *  Each scenario drives the REAL AuthorityArbiter, MixerStage and
 *  ActuatorStage -- the same objects the apps run -- and asserts
 *  the expected stop reason at the actuator. Nothing is mocked
 *  except the clock and the wire.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_SCENARIOS_HEADER
#define BB_SCENARIOS_HEADER

#include <string>
#include <vector>

namespace bb { namespace scenarios {

struct Result
{
  std::string name;
  bool        passed = false;
  std::string detail;
};

// Run every scenario. Returns one Result each; `verbose` prints a
// cycle-by-cycle trace for diagnosis.
std::vector<Result> run_all(bool verbose);

}} // namespace

#endif
