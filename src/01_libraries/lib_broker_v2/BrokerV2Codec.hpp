#pragma once

#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace broker_v2 {

struct FieldValue {
  bool is_double = false;
  double dval = 0.0;
  std::string sval;
};

using FieldMap = std::map<std::string, FieldValue>;
using NameMap = std::map<std::string, std::string>;

inline std::string trim(const std::string &in) {
  size_t first = 0;
  while ((first < in.size()) && std::isspace(static_cast<unsigned char>(in[first]))) {
    first++;
  }
  size_t last = in.size();
  while ((last > first) && std::isspace(static_cast<unsigned char>(in[last - 1]))) {
    last--;
  }
  return in.substr(first, last - first);
}

inline std::vector<std::string> split(const std::string &in, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : in) {
    if (ch == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  out.push_back(cur);
  return out;
}

inline bool parseDouble(const std::string &txt, double &out) {
  char *endptr = nullptr;
  const double v = std::strtod(txt.c_str(), &endptr);
  if ((endptr == txt.c_str()) || (endptr == nullptr) || (*endptr != '\0')) {
    return false;
  }
  out = v;
  return true;
}

inline std::string toHexByte(unsigned char value) {
  const char digits[] = "0123456789ABCDEF";
  std::string out;
  out.push_back(digits[(value >> 4U) & 0x0FU]);
  out.push_back(digits[value & 0x0FU]);
  return out;
}

inline int fromHexNibble(char c) {
  if ((c >= '0') && (c <= '9')) {
    return c - '0';
  }
  if ((c >= 'A') && (c <= 'F')) {
    return 10 + (c - 'A');
  }
  if ((c >= 'a') && (c <= 'f')) {
    return 10 + (c - 'a');
  }
  return -1;
}

inline std::string encodeString(const std::string &in) {
  std::string out;
  out.reserve(in.size() * 2);
  for (unsigned char c : in) {
    if (std::isalnum(c) || (c == '_') || (c == '-') || (c == '.') || (c == ':')) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('%');
      out += toHexByte(c);
    }
  }
  return out;
}

inline bool decodeString(const std::string &in, std::string &out) {
  out.clear();
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); i++) {
    if (in[i] != '%') {
      out.push_back(in[i]);
      continue;
    }
    if ((i + 2) >= in.size()) {
      return false;
    }
    const int hi = fromHexNibble(in[i + 1]);
    const int lo = fromHexNibble(in[i + 2]);
    if ((hi < 0) || (lo < 0)) {
      return false;
    }
    const unsigned char byte_val = static_cast<unsigned char>((hi << 4U) | lo);
    out.push_back(static_cast<char>(byte_val));
    i += 2;
  }
  return true;
}

inline std::string serializeFrame(const FieldMap &fields) {
  std::ostringstream oss;
  oss << "<";
  bool first = true;
  for (const auto &entry : fields) {
    if (!first) {
      oss << "|";
    }
    first = false;
    oss << entry.first << "=";
    if (entry.second.is_double) {
      oss << "D:" << std::setprecision(16) << entry.second.dval;
    } else {
      oss << "S:" << encodeString(entry.second.sval);
    }
  }
  oss << ">\n";
  return oss.str();
}

inline bool parseFrame(const std::string &msg, FieldMap &fields, std::string &error) {
  fields.clear();
  error.clear();

  std::string payload = trim(msg);
  if (payload.empty()) {
    error = "empty payload";
    return false;
  }
  if ((payload.front() == '<') && (payload.back() == '>')) {
    payload = payload.substr(1, payload.size() - 2);
  }

  const std::vector<std::string> parts = split(payload, '|');
  for (const std::string &raw_part : parts) {
    const std::string part = trim(raw_part);
    if (part.empty()) {
      continue;
    }

    const size_t eq_pos = part.find('=');
    if ((eq_pos == std::string::npos) || (eq_pos == 0) || (eq_pos >= (part.size() - 1))) {
      error = "bad key=value token";
      return false;
    }

    const std::string key = part.substr(0, eq_pos);
    const std::string encoded = part.substr(eq_pos + 1);
    if (encoded.size() < 2U || encoded[1] != ':') {
      error = "missing type prefix";
      return false;
    }

    FieldValue fv;
    if (encoded[0] == 'D') {
      fv.is_double = true;
      if (!parseDouble(encoded.substr(2), fv.dval)) {
        error = "invalid numeric field";
        return false;
      }
    } else if (encoded[0] == 'S') {
      fv.is_double = false;
      if (!decodeString(encoded.substr(2), fv.sval)) {
        error = "invalid encoded string field";
        return false;
      }
    } else {
      error = "unknown type prefix";
      return false;
    }
    fields[key] = fv;
  }

  return true;
}

inline NameMap parseNameMap(const std::string &spec) {
  NameMap mapping;
  const std::vector<std::string> tokens = split(spec, ',');
  for (const std::string &token_raw : tokens) {
    const std::string token = trim(token_raw);
    if (token.empty()) {
      continue;
    }
    const size_t colon_pos = token.find(':');
    if (colon_pos == std::string::npos) {
      mapping[token] = token;
      continue;
    }
    const std::string src = trim(token.substr(0, colon_pos));
    const std::string dst = trim(token.substr(colon_pos + 1));
    if (!src.empty() && !dst.empty()) {
      mapping[src] = dst;
    }
  }
  return mapping;
}

inline std::set<std::string> parseNameSet(const std::string &spec) {
  std::set<std::string> names;
  const std::vector<std::string> tokens = split(spec, ',');
  for (const std::string &token_raw : tokens) {
    const std::string token = trim(token_raw);
    if (!token.empty()) {
      names.insert(token);
    }
  }
  return names;
}

inline bool parseBool(const std::string &in, bool &out) {
  std::string norm;
  norm.reserve(in.size());
  for (char c : in) {
    norm.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  norm = trim(norm);
  if ((norm == "true") || (norm == "on") || (norm == "yes") || (norm == "1")) {
    out = true;
    return true;
  }
  if ((norm == "false") || (norm == "off") || (norm == "no") || (norm == "0")) {
    out = false;
    return true;
  }
  return false;
}

inline std::string remapName(const std::string &src, const NameMap &mapping) {
  const auto it = mapping.find(src);
  if (it == mapping.end()) {
    return src;
  }
  return it->second;
}

}  // namespace broker_v2

