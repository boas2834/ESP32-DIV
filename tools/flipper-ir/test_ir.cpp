// Host-side unit tests for the Flipper .ir parser and protocol encoders.
// Build & run:  g++ -std=c++17 -I../../ESP32-DIV test_ir.cpp -o test_ir && ./test_ir
//
// These exercise the hardware-independent core (ir_encoders.h,
// ir_flipper_parse.h). On-device behaviour (TFT UI, IRsend) is not covered
// here and must be tested on real hardware.
#define IR_ENCODERS_IMPL
#include "ir_encoders.h"
#include "ir_flipper_parse.h"

#include <cassert>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; } \
  else { std::printf("  ok:   %s\n", msg); } } while (0)

// A LineReader backed by an in-memory string.
static IrFlip::LineReader fromString(const std::string& s, std::shared_ptr<std::istringstream>& keep) {
  keep = std::make_shared<std::istringstream>(s);
  auto st = keep;
  return [st](std::string& line) -> bool {
    if (!std::getline(*st, line)) return false;
    return true;
  };
}

static uint32_t sumTimings(const std::vector<uint16_t>& t) {
  uint32_t s = 0; for (auto v : t) s += v; return s;
}

static void testParseHex() {
  std::printf("[parseHex4]\n");
  uint8_t out[4];
  IrFlip::parseHex4("04 00 00 00", out);
  CHECK(out[0] == 0x04 && out[1] == 0 && out[2] == 0 && out[3] == 0, "little-endian single byte");
  IrFlip::parseHex4("BF 40 1A 00", out);
  CHECK(out[0] == 0xBF && out[1] == 0x40 && out[2] == 0x1A && out[3] == 0, "multi-byte hex");
  IrFlip::parseHex4("ff ff", out);
  CHECK(out[0] == 0xFF && out[1] == 0xFF && out[2] == 0 && out[3] == 0, "lowercase + short");
}

static void testNEC() {
  std::printf("[NEC encode]\n");
  uint8_t a[4] = {0x04,0,0,0}, c[4] = {0x08,0,0,0};
  auto s = IrEnc::encode(IrEnc::Proto::NEC, a, c);
  CHECK(s.ok, "encodes");
  // header(2) + 32 bits * 2 + 1 stop = 67 entries
  CHECK(s.timings.size() == 67, "67 timing entries");
  CHECK(s.timings[0] == 9000 && s.timings[1] == 4500, "9000/4500 header");
  CHECK(s.frequencyHz == 38000, "38 kHz carrier");
  // addr=0x04 LSB-first: bits 0,0,1,0,0,0,0,0 -> 3rd bit (index2) is the '1'.
  // Entry layout after header: index 2 = bit0 mark, 3 = bit0 space, ...
  // bit0=0 -> space 560 at idx3; bit2=1 -> space 1690 at idx 2+2*2+1 = 7.
  CHECK(s.timings[3] == 560,  "addr bit0 = 0 (short space)");
  CHECK(s.timings[7] == 1690, "addr bit2 = 1 (long space)");
  // inverted address byte ~0x04 = 0xFB: bit0=1 -> long space at idx 2+16+1=19
  CHECK(s.timings[19] == 1690, "inverted-addr bit0 = 1");
  CHECK(s.timings.back() == 560, "final stop mark");
}

static void testNECext() {
  std::printf("[NECext encode]\n");
  uint8_t a[4] = {0xBF,0x40,0,0}, c[4] = {0x12,0x00,0,0};
  auto s = IrEnc::encode(IrEnc::Proto::NECext, a, c);
  CHECK(s.ok, "encodes");
  // header(2) + 32 bits*2 + 1 = 67
  CHECK(s.timings.size() == 67, "67 timing entries (16+16 bits)");
}

static void testSamsung() {
  std::printf("[Samsung32 encode]\n");
  uint8_t a[4] = {0x07,0,0,0}, c[4] = {0x02,0,0,0};
  auto s = IrEnc::encode(IrEnc::Proto::Samsung32, a, c);
  CHECK(s.ok, "encodes");
  CHECK(s.timings[0] == 4500 && s.timings[1] == 4500, "4500/4500 header");
  CHECK(s.timings.size() == 67, "67 timing entries");
}

static void testSIRC() {
  std::printf("[SIRC encode]\n");
  uint8_t a[4] = {0x01,0,0,0}, c[4] = {0x15,0,0,0};  // Sony TV power
  auto s = IrEnc::encode(IrEnc::Proto::SIRC, a, c);
  CHECK(s.ok, "encodes");
  CHECK(s.frequencyHz == 40000, "40 kHz carrier");
  CHECK(s.timings[0] == 2400 && s.timings[1] == 600, "2400/600 header");
  // 12 bits -> header(2) + 12*2 = 26 entries
  CHECK(s.timings.size() == 26, "26 timing entries (12-bit)");
  auto s20 = IrEnc::encode(IrEnc::Proto::SIRC20, a, c);
  CHECK(s20.timings.size() == 2 + 20 * 2, "SIRC20 = 42 entries");
}

static void testRC5() {
  std::printf("[RC5 encode]\n");
  uint8_t a[4] = {0x00,0,0,0}, c[4] = {0x01,0,0,0};
  auto s = IrEnc::encode(IrEnc::Proto::RC5, a, c);
  CHECK(s.ok, "encodes");
  CHECK(s.frequencyHz == 36000, "36 kHz carrier");
  // 14 Manchester bits => total duration = 14 * 2 * 889 us (minus the one
  // dropped leading space half-cell).
  uint32_t total = sumTimings(s.timings);
  CHECK(total == (uint32_t)(14 * 2 * 889 - 889), "total duration = 27 half-bits");
}

static void testUnsupported() {
  std::printf("[unsupported proto]\n");
  uint8_t a[4] = {0}, c[4] = {0};
  auto s = IrEnc::encode(IrEnc::Proto::RC6, a, c);
  CHECK(!s.ok, "RC6 not synthesised -> ok=false");
  CHECK(!IrEnc::isEncodable(IrEnc::Proto::Kaseikyo), "Kaseikyo not encodable");
  CHECK(IrEnc::isEncodable(IrEnc::Proto::NEC), "NEC encodable");
}

static void testParseFile() {
  std::printf("[parse .ir file]\n");
  const std::string f =
    "Filetype: IR library file\n"
    "Version: 1\n"
    "# comment\n"
    "#\n"
    "name: Power\n"
    "type: parsed\n"
    "protocol: SIRC\n"
    "address: 01 00 00 00\n"
    "command: 15 00 00 00\n"
    "#\n"
    "name: Noise\n"
    "type: raw\n"
    "frequency: 38000\n"
    "duty_cycle: 0.330000\n"
    "data: 278 1811 277 788 246 794\n"
    "#\n";
  std::shared_ptr<std::istringstream> keep;
  IrFlip::Reader r(fromString(f, keep));
  IrFlip::Record rec;
  bool got1 = r.next(rec);
  CHECK(got1 && rec.name == "Power", "record 1 name");
  CHECK(!rec.raw && rec.proto == IrEnc::Proto::SIRC, "record 1 parsed SIRC");
  CHECK(rec.addr[0] == 0x01 && rec.cmd[0] == 0x15, "record 1 addr/cmd");
  CHECK(rec.sendable(), "record 1 sendable");
  bool got2 = r.next(rec);
  CHECK(got2 && rec.name == "Noise" && rec.raw, "record 2 raw");
  CHECK(rec.frequencyHz == 38000, "record 2 frequency");
  CHECK(rec.rawTimings.size() == 6, "record 2 six timings");
  CHECK(rec.rawTimings[0] == 278 && rec.rawTimings[5] == 794, "record 2 timing values");
  bool got3 = r.next(rec);
  CHECK(!got3, "no third record (EOF)");
}

int main() {
  testParseHex();
  testNEC();
  testNECext();
  testSamsung();
  testSIRC();
  testRC5();
  testUnsupported();
  testParseFile();
  std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
              g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
