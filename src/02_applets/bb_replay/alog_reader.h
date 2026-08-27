/*************************************************************
 *  alog_reader -- minimal MOOS .alog reader for offline replay.
 *
 *  Deliberately small. It does not depend on MOOS, so the replay
 *  harness builds and runs on any laptop, which is the whole
 *  point of Tier 2 (docs/control_refactor_plan.md section 8.2).
 *
 *  .alog line format:
 *      <time>  <VARIABLE>  <source>  <value>
 *  whitespace-separated, comments begin with '%'.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_ALOG_READER_HEADER
#define BB_ALOG_READER_HEADER

#include <map>
#include <set>
#include <string>
#include <vector>

namespace bb {

struct AlogSample
{
  double      time = 0.0;
  std::string source;
  std::string text;     // raw value, always present
  double      value = 0.0;
  bool        numeric = false;
};

// One variable's samples, in file order (which is time order).
typedef std::vector<AlogSample> AlogSeries;

class AlogLog
{
 public:
  // Read `path`, keeping only the variables named in `wanted`.
  // An empty `wanted` keeps everything, which on a 250 MB log is
  // usually a mistake -- name what you need.
  bool load(const std::string& path, const std::set<std::string>& wanted,
            std::string& error);

  bool has(const std::string& var) const;
  const AlogSeries& series(const std::string& var) const;

  // Most recent sample at or before `t`. Returns nullptr if the
  // series is empty or `t` precedes its first sample.
  //
  // This is zero-order hold, which is what a MOOS consumer
  // actually sees: a subscriber holds the last value it received
  // until the next one arrives.
  const AlogSample* at_or_before(const std::string& var, double t) const;

  double first_time() const { return m_first_time; }
  double last_time()  const { return m_last_time;  }
  long   lines_read() const { return m_lines_read; }

  std::vector<std::string> variables() const;

 private:
  std::map<std::string, AlogSeries> m_series;
  AlogSeries m_empty;
  double m_first_time = 0.0;
  double m_last_time  = 0.0;
  long   m_lines_read = 0;
};

} // namespace bb

#endif
