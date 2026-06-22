/*************************************************************
      Name: Ray Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_AthensLogger/BB_AthensLogger.cpp
   Last Ed: 2026-06-22
     Brief:
        Front-seat data logger for the BlueBoat fleet. A pure
        sink: subscribes to the front-seat MOOSDB (wildcard by
        default) and appends every variable update to a single
        per-day, pipe-delimited time-series file in arrival
        order, preserving each message's generation time. It
        never publishes and never touches the control path, so
        a crash is harmless to the vehicle. Missions/sessions
        are discriminated after the fact by the logged
        MISSION_HASH rows.

        Record format (one line per variable update):
            time|var|src|type|value
        - time : message generation time (Unix epoch seconds)
        - var  : MOOS variable name
        - src  : originating app (helps tell relayed back-seat
                 data, posted by iFrontSeatBroker, from native)
        - type : 'd' (double) or 's' (string)
        - value: raw value, written LAST so embedded commas are
                 harmless; split with maxsplit=4 in post-proc.
        Comment lines begin with '#'. Files are append-only and
        roll at the local (or UTC) day boundary.
*************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BB_AthensLogger.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <cstdio>
#include <cstring>

using namespace std;

//---------------------------------------------------------
// Constructor()

BB_AthensLogger::BB_AthensLogger()
{
  m_debug         = false;
  m_debug_stream  = nullptr;
  memset(m_fname, 0, m_fname_buff_size);

  m_vname         = "unknown";
  m_log_dir       = "/home/pi/bb_daily_logs";
  m_wildcard      = true;
  m_use_utc       = false;     // filename day boundary in local time
  m_value_digits  = 8;

  m_file_ok       = false;
  m_lines_logged  = 0;
  m_lines_omitted = 0;

  // Default omit set: pure infrastructure noise and bulky appcast
  // machinery. Everything else (including duplicative nav data) is
  // kept, per design. Override/extend via 'omit' / 'keep' config.
  m_omit_patterns.push_back("DB_*");
  m_omit_patterns.push_back("APPCAST");
  m_omit_patterns.push_back("APPCAST_REQ*");
  m_omit_patterns.push_back("APP_LOG");
  m_omit_patterns.push_back("REALMCAST*");
  m_omit_patterns.push_back("*_ITER_GAP");
  m_omit_patterns.push_back("*_ITER_LEN");
  m_omit_patterns.push_back("PSHARE_*_SUMMARY");
  m_omit_patterns.push_back("*_STATUS");
  m_omit_patterns.push_back("*_PID");

  // Useful aggregates that would otherwise be caught by an omit
  // pattern (e.g. BB_STATUS matches "*_STATUS").
  m_keep_vars.insert("BB_STATUS");
}

//---------------------------------------------------------
// Destructor

BB_AthensLogger::~BB_AthensLogger()
{
  if(m_ofs.is_open()) {
    m_ofs << "# === session end | utc=" << doubleToStringX(MOOSTime(), 2)
          << " | lines=" << m_lines_logged << " ===" << endl;
    m_ofs.flush();
    m_ofs.close();
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BB_AthensLogger::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "APPCAST_REQ") // handled by AppCastingMOOSApp
      continue;

    m_vars_seen.insert(key);

    if(key == "MISSION_HASH")
      m_cur_mhash = msg.GetString();

    if(!shouldLog(key)) {
      m_lines_omitted++;
      continue;
    }

    if(msg.IsDouble())
      writeRecord(key, msg.GetSource(), 'd',
                  doubleToStringX(msg.GetDouble(), (int)m_value_digits),
                  msg.GetTime());
    else
      writeRecord(key, msg.GetSource(), 's',
                  sanitizeString(msg.GetString()), msg.GetTime());
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: shouldLog()
//   Keep-list wins over omit-list. When wildcard is off, only
//   explicitly listed variables are logged.

bool BB_AthensLogger::shouldLog(const string &key) const
{
  if(m_keep_vars.count(key))
    return(true);

  if(!m_wildcard)
    return(m_log_vars.count(key) > 0);

  for(unsigned int i=0; i<m_omit_patterns.size(); i++)
    if(matchPattern(key, m_omit_patterns[i]))
      return(false);

  return(true);
}

//---------------------------------------------------------
// Procedure: matchPattern()
//   Minimal glob: supports a single leading and/or trailing '*'.
//   "DB_*" prefix, "*_ITER_GAP" suffix, "*FOO*" contains, "X" exact.

bool BB_AthensLogger::matchPattern(const string &name, const string &pat) const
{
  bool star_front = (!pat.empty() && pat.front() == '*');
  bool star_back  = (!pat.empty() && pat.back()  == '*');

  string core = pat;
  if(star_front) core = core.substr(1);
  if(star_back && !core.empty()) core = core.substr(0, core.size()-1);

  if(!star_front && !star_back)
    return(name == pat);
  if(star_front && star_back)
    return(name.find(core) != string::npos);
  if(star_back) // prefix match
    return(name.compare(0, core.size(), core) == 0);
  // star_front -> suffix match
  if(core.size() > name.size())
    return(false);
  return(name.compare(name.size()-core.size(), core.size(), core) == 0);
}

//---------------------------------------------------------
// Procedure: sanitizeString()
//   Strip newlines/carriage returns so each record stays on one
//   line. Embedded '|' are left intact (value is the last field).

string BB_AthensLogger::sanitizeString(const string &in) const
{
  string out = in;
  for(unsigned int i=0; i<out.size(); i++)
    if(out[i] == '\n' || out[i] == '\r')
      out[i] = ' ';
  return(out);
}

//---------------------------------------------------------
// Procedure: dateStamp() / dailyFilePath()

string BB_AthensLogger::dateStamp(double mtime) const
{
  time_t t = (time_t)mtime;
  struct tm tmv;
  if(m_use_utc) gmtime_r(&t, &tmv);
  else          localtime_r(&t, &tmv);
  char buf[16];
  strftime(buf, sizeof(buf), "%Y%m%d", &tmv);
  return(string(buf));
}

string BB_AthensLogger::dailyFilePath(double mtime) const
{
  return(m_log_dir + "/LOG_" + m_vname + "_" + dateStamp(mtime) + ".psv");
}

//---------------------------------------------------------
// Procedure: ensureFileForTime()
//   Opens (append) the daily file for the given time, rolling at
//   the day boundary. A new/empty file gets a header; reopened
//   files get a session marker. Failure is non-fatal (sink only).

void BB_AthensLogger::ensureFileForTime(double mtime)
{
  string date = dateStamp(mtime);
  if(m_ofs.is_open() && date == m_open_date)
    return;

  if(m_ofs.is_open()) {
    m_ofs.flush();
    m_ofs.close();
  }

  string path = dailyFilePath(mtime);
  m_ofs.open(path.c_str(), ios::out | ios::app);
  if(!m_ofs.is_open()) {
    m_file_ok = false;
    reportRunWarning("Could not open log file: " + path);
    return;
  }

  m_file_ok   = true;
  m_open_date = date;
  m_open_path = path;

  if(m_ofs.tellp() == (streampos)0)               // fresh file
    m_ofs << "# pBB_AthensLogger | vname=" << m_vname << "\n"
          << "time|var|src|type|value" << endl;

  m_ofs << "# === session start | vname=" << m_vname
        << " | utc=" << doubleToStringX(MOOSTime(), 2)
        << " | mhash=" << (m_cur_mhash.empty() ? "n/a" : m_cur_mhash)
        << " ===" << endl;
}

//---------------------------------------------------------
// Procedure: writeRecord()

void BB_AthensLogger::writeRecord(const string &key, const string &src,
                                  char type, const string &value, double mtime)
{
  ensureFileForTime(mtime);
  if(!m_file_ok)
    return;

  m_ofs << doubleToStringX(mtime, 4) << "|" << key << "|" << src
        << "|" << type << "|" << value << "\n";
  m_lines_logged++;
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool BB_AthensLogger::dbg_print(const char *format, ...)
{
  if(m_debug == true) {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if(m_debug_stream != nullptr) {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      va_end(args);
      return true;
    }
    else {
      va_end(args);
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BB_AthensLogger::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BB_AthensLogger::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(m_ofs.is_open())
    m_ofs.flush();                  // bound data loss on crash to one tick
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BB_AthensLogger::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = stripBlankEnds(line);

    bool handled = false;
    if(param == "vname") {
      m_vname = value;
      handled = true;
    }
    else if(param == "log_dir") {
      m_log_dir = value;
      handled = true;
    }
    else if(param == "wildcard") {
      handled = setBooleanOnString(m_wildcard, value);
    }
    else if(param == "utc") {
      handled = setBooleanOnString(m_use_utc, value);
    }
    else if(param == "value_digits") {
      if(isNumber(value)) { m_value_digits = (unsigned int)atoi(value.c_str()); handled = true; }
    }
    else if(param == "log") {        // explicit var(s), comma-separated ok
      vector<string> vars = parseString(value, ',');
      for(unsigned int i=0; i<vars.size(); i++) {
        string v = stripBlankEnds(vars[i]);
        if(v != "") m_log_vars.insert(v);
      }
      handled = true;
    }
    else if(param == "keep") {       // exact names that override omit
      vector<string> vars = parseString(value, ',');
      for(unsigned int i=0; i<vars.size(); i++) {
        string v = stripBlankEnds(vars[i]);
        if(v != "") m_keep_vars.insert(v);
      }
      handled = true;
    }
    else if(param == "omit") {       // replace default omit patterns
      m_omit_patterns.clear();
      vector<string> pats = parseString(value, ',');
      for(unsigned int i=0; i<pats.size(); i++) {
        string v = stripBlankEnds(pats[i]);
        if(v != "") m_omit_patterns.push_back(v);
      }
      handled = true;
    }
    else if(param == "debug") {
      m_debug = (tolower(value) == "true");
      if(m_debug) {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, 0, m_fname_buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, 0, m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%H-%M-%S", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_app_name.c_str(), fmt);
      }
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Best-effort create of the log directory (parents too). Non-fatal:
  // if it cannot be made, the file open later just reports a warning.
  if(m_log_dir != "") {
    string acc;
    vector<string> parts = parseString(m_log_dir, '/');
    if(m_log_dir[0] == '/') acc = "";          // absolute
    for(unsigned int i=0; i<parts.size(); i++) {
      if(parts[i] == "") continue;
      acc += "/" + parts[i];
      mkdir(acc.c_str(), 0775);                 // ignore EEXIST
    }
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BB_AthensLogger::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  if(m_wildcard)
    Register("*", "*", 0);            // every variable, every source
  else {
    set<string>::iterator q;
    for(q=m_log_vars.begin(); q!=m_log_vars.end(); q++)
      Register(*q, 0);
    Register("MISSION_HASH", 0);      // always track mhash for markers
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool BB_AthensLogger::buildReport()
{
  m_msgs << "Vehicle:     " << m_vname << endl;
  m_msgs << "Log file:    " << (m_open_path == "" ? "(none yet)" : m_open_path) << endl;
  m_msgs << "File OK:      " << (m_file_ok ? "yes" : "no") << endl;
  m_msgs << "Mode:        " << (m_wildcard ? "wildcard (*)" : "explicit list") << endl;
  m_msgs << "Day stamp:   " << (m_use_utc ? "UTC" : "local") << endl;
  m_msgs << "MISSION_HASH:" << (m_cur_mhash.empty() ? " (not seen)" : " " + m_cur_mhash) << endl;
  m_msgs << endl;
  m_msgs << "Vars seen:   " << m_vars_seen.size() << endl;
  m_msgs << "Lines logged:" << m_lines_logged << endl;
  m_msgs << "Lines omitted:" << m_lines_omitted << endl;

  return(true);
}
