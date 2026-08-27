#include "alog_reader.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace bb {

namespace {

bool parse_numeric(const std::string& s, double& out)
{
  if (s.empty()) return false;
  const char* b = s.c_str();
  char* e = nullptr;
  const double v = strtod(b, &e);
  if (e == b || *e != '\0') return false;
  if (!std::isfinite(v)) return false;
  out = v;
  return true;
}

} // namespace

bool AlogLog::load(const std::string& path, const std::set<std::string>& wanted,
                   std::string& error)
{
  FILE* f = fopen(path.c_str(), "r");
  if (!f) { error = "cannot open " + path; return false; }

  const bool keep_all = wanted.empty();

  // Logs run to hundreds of MB; a generous line buffer avoids
  // splitting long APPCAST values across reads.
  std::vector<char> buf(1 << 16);
  bool first = true;

  while (fgets(buf.data(), (int)buf.size(), f)) {
    ++m_lines_read;
    const char* p = buf.data();
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '%' || *p == '\n' || *p == '\0') continue;

    // time
    char* e = nullptr;
    const double t = strtod(p, &e);
    if (e == p) continue;
    p = e;

    // variable
    while (*p == ' ' || *p == '\t') ++p;
    const char* vb = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (p == vb) continue;
    const std::string var(vb, p - vb);

    if (!keep_all && wanted.find(var) == wanted.end()) continue;

    // source
    while (*p == ' ' || *p == '\t') ++p;
    const char* sb = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    const std::string src(sb, p - sb);

    // value: rest of line, trimmed
    while (*p == ' ' || *p == '\t') ++p;
    const char* val_begin = p;
    const char* val_end = val_begin;
    while (*val_end && *val_end != '\n' && *val_end != '\r') ++val_end;
    while (val_end > val_begin && *(val_end - 1) == ' ') --val_end;

    AlogSample s;
    s.time = t;
    s.source = src;
    s.text.assign(val_begin, val_end - val_begin);
    s.numeric = parse_numeric(s.text, s.value);

    m_series[var].push_back(s);

    if (first) { m_first_time = t; first = false; }
    m_last_time = t;
  }
  fclose(f);

  // .alog is written in time order, but a log stitched by hand
  // may not be. Sorting is cheap insurance and makes
  // at_or_before()'s binary search sound.
  for (std::map<std::string, AlogSeries>::iterator it = m_series.begin();
       it != m_series.end(); ++it) {
    std::stable_sort(it->second.begin(), it->second.end(),
                     [](const AlogSample& a, const AlogSample& b) {
                       return a.time < b.time;
                     });
  }
  return true;
}

bool AlogLog::has(const std::string& var) const
{
  std::map<std::string, AlogSeries>::const_iterator it = m_series.find(var);
  return it != m_series.end() && !it->second.empty();
}

const AlogSeries& AlogLog::series(const std::string& var) const
{
  std::map<std::string, AlogSeries>::const_iterator it = m_series.find(var);
  return (it == m_series.end()) ? m_empty : it->second;
}

const AlogSample* AlogLog::at_or_before(const std::string& var, double t) const
{
  const AlogSeries& s = series(var);
  if (s.empty()) return nullptr;

  size_t lo = 0, hi = s.size();
  while (lo < hi) {
    const size_t mid = (lo + hi) / 2;
    if (s[mid].time <= t) lo = mid + 1;
    else hi = mid;
  }
  if (lo == 0) return nullptr;
  return &s[lo - 1];
}

std::vector<std::string> AlogLog::variables() const
{
  std::vector<std::string> out;
  for (std::map<std::string, AlogSeries>::const_iterator it = m_series.begin();
       it != m_series.end(); ++it)
    out.push_back(it->first);
  return out;
}

} // namespace bb
