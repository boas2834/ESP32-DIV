#pragma once
//
// ir_flipper_parse.h — pure parser for Flipper Zero ".ir" library files.
//
// No Arduino/SD dependency: the caller supplies a line reader, so the same
// code parses from an SD File on the device and from a std::ifstream in the
// host unit test. Records are separated by lines beginning with '#'.
//
// Flipper .ir record, parsed form:
//   name: Power
//   type: parsed
//   protocol: NEC
//   address: 04 00 00 00
//   command: 08 00 00 00
// raw form:
//   name: Power
//   type: raw
//   frequency: 38000
//   duty_cycle: 0.330000
//   data: 9000 4500 560 560 ...
//
#include <stdint.h>
#include <functional>
#include <string>
#include <vector>
#include "ir_encoders.h"

namespace IrFlip {

struct Record {
  std::string name;
  bool raw = false;
  bool valid = false;

  // parsed
  IrEnc::Proto proto = IrEnc::Proto::Unknown;
  std::string protoName;        // original string, for display
  uint8_t addr[4] = {0, 0, 0, 0};
  uint8_t cmd[4]  = {0, 0, 0, 0};

  // raw
  uint16_t frequencyHz = 38000;
  std::vector<uint16_t> rawTimings;

  // True if this record can actually be transmitted on this device.
  bool sendable() const {
    if (raw) return rawTimings.size() >= 2;
    return IrEnc::isEncodable(proto);
  }
};

// Reads a 4-byte little-endian hex field like "04 00 00 00".
inline void parseHex4(const std::string& v, uint8_t out[4]) {
  out[0] = out[1] = out[2] = out[3] = 0;
  int idx = 0;
  size_t i = 0;
  while (i < v.size() && idx < 4) {
    while (i < v.size() && (v[i] == ' ' || v[i] == '\t')) i++;
    if (i >= v.size()) break;
    unsigned byte = 0; int digits = 0;
    while (i < v.size() && v[i] != ' ' && v[i] != '\t' && digits < 2) {
      char ch = v[i];
      int d;
      if (ch >= '0' && ch <= '9') d = ch - '0';
      else if (ch >= 'a' && ch <= 'f') d = ch - 'a' + 10;
      else if (ch >= 'A' && ch <= 'F') d = ch - 'A' + 10;
      else break;
      byte = (byte << 4) | d; digits++; i++;
    }
    if (digits > 0) out[idx++] = (uint8_t)byte;
  }
}

// Splits "key: value" and trims. Returns false if no colon.
inline bool splitField(const std::string& line, std::string& key, std::string& val) {
  size_t c = line.find(':');
  if (c == std::string::npos) return false;
  key = line.substr(0, c);
  size_t s = c + 1;
  while (s < line.size() && (line[s] == ' ' || line[s] == '\t')) s++;
  size_t e = line.size();
  while (e > s && (line[e - 1] == '\r' || line[e - 1] == '\n' ||
                   line[e - 1] == ' '  || line[e - 1] == '\t')) e--;
  val = line.substr(s, e - s);
  // trim key trailing spaces
  size_t ke = key.size();
  while (ke > 0 && (key[ke - 1] == ' ' || key[ke - 1] == '\t')) ke--;
  key = key.substr(0, ke);
  return true;
}

inline void parseRawData(const std::string& v, std::vector<uint16_t>& out) {
  size_t i = 0;
  while (i < v.size()) {
    while (i < v.size() && (v[i] == ' ' || v[i] == '\t')) i++;
    uint32_t n = 0; int digits = 0;
    while (i < v.size() && v[i] >= '0' && v[i] <= '9') {
      n = n * 10 + (v[i] - '0'); digits++; i++;
    }
    if (digits > 0) out.push_back(n > 0xFFFF ? 0xFFFF : (uint16_t)n);
    else i++;  // skip any stray char
  }
}

// LineReader: fills `line` with the next line (without requiring a newline),
// returns false at EOF. A line beginning with '#' ends the current record.
using LineReader = std::function<bool(std::string& line)>;

// Streaming reader: call next() repeatedly; each true fills `rec` with one
// record. Keeps only one record in memory at a time (SD-friendly).
class Reader {
 public:
  explicit Reader(LineReader lr) : lr_(std::move(lr)) {}

  bool next(Record& rec) {
    rec = Record();
    bool haveAny = false;
    std::string line;
    // A pushed-back separator (from the previous record) is ignored; we just
    // keep reading field lines until the next separator or EOF.
    while (lr_(line)) {
      if (!line.empty() && (line[0] == '#')) {
        if (haveAny) { finalize(rec); return true; }
        continue;  // leading separators / comments before first record
      }
      // Skip the file header lines.
      std::string key, val;
      if (!splitField(line, key, val)) continue;
      if (key == "Filetype" || key == "Version") continue;
      haveAny = true;
      if (key == "name") rec.name = val;
      else if (key == "type") rec.raw = (val == "raw");
      else if (key == "protocol") {
        rec.protoName = val;
        rec.proto = IrEnc::protoFromName(val);
      }
      else if (key == "address")   parseHex4(val, rec.addr);
      else if (key == "command")   parseHex4(val, rec.cmd);
      else if (key == "frequency") rec.frequencyHz = (uint16_t)atoiSafe(val);
      else if (key == "data")      parseRawData(val, rec.rawTimings);
    }
    if (haveAny) { finalize(rec); return true; }
    return false;
  }

 private:
  static long atoiSafe(const std::string& v) {
    long n = 0; for (char ch : v) { if (ch < '0' || ch > '9') break; n = n * 10 + (ch - '0'); }
    return n;
  }
  static void finalize(Record& rec) {
    rec.valid = !rec.name.empty();
  }
  LineReader lr_;
};

} // namespace IrFlip
