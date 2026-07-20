/*************************************************************
      Name: Ray Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_AthensLogger/BB_AthensLogger.h
   Last Ed: 2026-06-22
     Brief:
        Front-seat data logger for the BlueBoat fleet. A pure
        sink: subscribes to the front-seat MOOSDB (wildcard by
        default), and appends every variable update to a single
        per-day, pipe-delimited time-series file in arrival
        order, preserving each message's generation time. Never
        publishes and never sits in the control path, so a crash
        cannot affect vehicle behavior. Sessions/missions are
        discriminated after the fact by the logged MISSION_HASH
        rows; daily files capture all data as students rotate
        through the back seats.
*************************************************************/

#ifndef BB_AthensLogger_HEADER
#define BB_AthensLogger_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <cstdarg> //va_list, va_start, va_end

class BB_AthensLogger : public AppCastingMOOSApp
{
 public:
   BB_AthensLogger();
   ~BB_AthensLogger();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);

 protected: // Logger helpers
   bool        shouldLog(const std::string &key) const;
   bool        matchPattern(const std::string &name, const std::string &pat) const;
   void        ensureFileForTime(double mtime);
   std::string dailyFilePath(double mtime) const;
   std::string dateStamp(double mtime) const;   // YYYYMMDD for filename
   void        writeRecord(const std::string &key, const std::string &src,
                           char type, const std::string &value, double mtime);
   std::string sanitizeString(const std::string &in) const;

 private: // Configuration variables

  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

  std::string  m_vname;        // vehicle name, used in the filename
  std::string  m_log_dir;      // launch-stable directory for daily files
  bool         m_wildcard;     // subscribe to "*" when true
  bool         m_use_utc;      // filename day boundary in UTC vs local
  unsigned int m_value_digits; // decimal digits when formatting doubles

  std::set<std::string>    m_log_vars;       // explicit vars (wildcard==false)
  std::set<std::string>    m_keep_vars;      // exact names that override omit
  std::vector<std::string> m_omit_patterns;  // glob patterns ('*' prefix/suffix)

 private: // State variables

  std::ofstream m_ofs;
  std::string   m_open_date;   // YYYYMMDD currently open
  std::string   m_open_path;   // full path currently open
  bool          m_file_ok;     // false if we failed to open (keep running)

  std::string   m_cur_mhash;   // last MISSION_HASH seen (for appcast/markers)
  unsigned long m_lines_logged;
  unsigned long m_lines_omitted;
  std::set<std::string> m_vars_seen;
};

#endif
