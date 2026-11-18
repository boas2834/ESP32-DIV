#pragma once
//
// ir_encoders.h — pure, hardware-independent encoders that turn a Flipper
// "parsed" IR signal (protocol + address + command) into a list of raw
// mark/space durations in microseconds, plus the carrier frequency.
//
// This header has NO Arduino/TFT/SD dependencies on purpose: it is compiled
// both into the firmware and into a host-side unit test (tools/flipper-ir).
// The firmware feeds the produced timings straight into IRsend::sendRaw(),
// which sidesteps any bit-order mismatch between IR libraries: we own the
// exact waveform.
//
// Timings follow the same protocol definitions the Flipper uses, so a signal
// captured on a Flipper and exported as .ir reproduces the original remote.
//
#include <stdint.h>
#include <string>
#include <vector>

namespace IrEnc {

// Result of encoding one button press.
struct Signal {
  uint16_t frequencyHz = 38000;   // carrier
  std::vector<uint16_t> timings;  // mark, space, mark, space, ... (microseconds)
  bool ok = false;                // false => protocol unsupported / bad input
};

// Supported parsed protocols. Names match the Flipper .ir "protocol:" field.
enum class Proto {
  Unknown,
  NEC,        // 8-bit addr + 8-bit cmd, inverted repeats
  NECext,     // 16-bit addr + 16-bit cmd, no inversion
  NEC42,      // 13-bit addr + 8-bit cmd
  Samsung32,  // 8-bit addr + 8-bit cmd
  SIRC,       // Sony 12-bit (7 cmd + 5 addr)
  SIRC15,     // Sony 15-bit (7 cmd + 8 addr)
  SIRC20,     // Sony 20-bit (7 cmd + 5 addr + 8 ext)
  RC5,        // Philips 14-bit Manchester
  RC5X,       // RC5 extended (command > 63)
  RC6,        // Philips RC6 mode 0, 20-bit
  Kaseikyo,   // Panasonic family, 48-bit
};

Proto protoFromName(const std::string& name);
const char* protoName(Proto p);

// True if encode() can produce a waveform for this protocol.
bool isEncodable(Proto p);

// addr / cmd are the little-endian byte arrays straight from the .ir file
// ("address: 01 00 00 00" -> {0x01,0,0,0}). Up to 4 bytes each are honoured.
Signal encode(Proto p, const uint8_t addr[4], const uint8_t cmd[4]);

// ---- helpers reused by the encoders (exposed for testing) ----

// Append one pulse-distance-coded bit (used by NEC/Samsung/Kaseikyo):
// a fixed mark, then a short space for 0 or a long space for 1.
void pushPdcBit(std::vector<uint16_t>& t, bool bit,
                uint16_t mark, uint16_t zeroSpace, uint16_t oneSpace);

// Append n bits of value, LSB first, pulse-distance coded.
void pushPdcByteLSB(std::vector<uint16_t>& t, uint32_t value, int nbits,
                    uint16_t mark, uint16_t zeroSpace, uint16_t oneSpace);

} // namespace IrEnc

// ------------------------------------------------------------------
// Implementation (header-only so the host test links without a .cpp).
// ------------------------------------------------------------------
#ifdef IR_ENCODERS_IMPL
namespace IrEnc {

Proto protoFromName(const std::string& n) {
  if (n == "NEC")       return Proto::NEC;
  if (n == "NECext")    return Proto::NECext;
  if (n == "NEC42")     return Proto::NEC42;
  if (n == "NEC42ext")  return Proto::NEC42;   // treated as NEC42 payload
  if (n == "Samsung32") return Proto::Samsung32;
  if (n == "SIRC")      return Proto::SIRC;
  if (n == "SIRC15")    return Proto::SIRC15;
  if (n == "SIRC20")    return Proto::SIRC20;
  if (n == "RC5")       return Proto::RC5;
  if (n == "RC5X")      return Proto::RC5X;
  if (n == "RC6")       return Proto::RC6;
  if (n == "Kaseikyo")  return Proto::Kaseikyo;
  return Proto::Unknown;
}

const char* protoName(Proto p) {
  switch (p) {
    case Proto::NEC:       return "NEC";
    case Proto::NECext:    return "NECext";
    case Proto::NEC42:     return "NEC42";
    case Proto::Samsung32: return "Samsung32";
    case Proto::SIRC:      return "SIRC";
    case Proto::SIRC15:    return "SIRC15";
    case Proto::SIRC20:    return "SIRC20";
    case Proto::RC5:       return "RC5";
    case Proto::RC5X:      return "RC5X";
    case Proto::RC6:       return "RC6";
    case Proto::Kaseikyo:  return "Kaseikyo";
    default:               return "Unknown";
  }
}

bool isEncodable(Proto p) {
  // Only protocols with an implemented encoder below. RC6/Kaseikyo are
  // recognised by name (for display) but not yet synthesised; such a parsed
  // signal is skipped unless the .ir file also carries a raw variant.
  switch (p) {
    case Proto::NEC: case Proto::NECext: case Proto::NEC42:
    case Proto::Samsung32: case Proto::SIRC: case Proto::SIRC15:
    case Proto::SIRC20: case Proto::RC5: case Proto::RC5X:
      return true;
    default:
      return false;
  }
}

void pushPdcBit(std::vector<uint16_t>& t, bool bit,
                uint16_t mark, uint16_t zeroSpace, uint16_t oneSpace) {
  t.push_back(mark);
  t.push_back(bit ? oneSpace : zeroSpace);
}

void pushPdcByteLSB(std::vector<uint16_t>& t, uint32_t value, int nbits,
                    uint16_t mark, uint16_t zeroSpace, uint16_t oneSpace) {
  for (int i = 0; i < nbits; ++i)
    pushPdcBit(t, (value >> i) & 1u, mark, zeroSpace, oneSpace);
}

// ---- NEC family (pulse-distance, LSB first) ----
// Timings per the standard NEC spec, matching Flipper's irda NEC protocol.
namespace {
constexpr uint16_t NEC_HDR_MARK  = 9000;
constexpr uint16_t NEC_HDR_SPACE = 4500;
constexpr uint16_t NEC_BIT_MARK  = 560;
constexpr uint16_t NEC_ZERO      = 560;
constexpr uint16_t NEC_ONE       = 1690;

Signal encodeNEC(const uint8_t addr[4], const uint8_t cmd[4]) {
  Signal s; s.frequencyHz = 38000;
  auto& t = s.timings;
  t.push_back(NEC_HDR_MARK); t.push_back(NEC_HDR_SPACE);
  uint8_t a = addr[0], c = cmd[0];
  pushPdcByteLSB(t, a, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, (uint8_t)~a, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, c, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, (uint8_t)~c, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  t.push_back(NEC_BIT_MARK); // final stop mark
  s.ok = true;
  return s;
}

Signal encodeNECext(const uint8_t addr[4], const uint8_t cmd[4]) {
  // 16-bit address and 16-bit command, each sent LSB first, no inversion.
  Signal s; s.frequencyHz = 38000;
  auto& t = s.timings;
  t.push_back(NEC_HDR_MARK); t.push_back(NEC_HDR_SPACE);
  uint16_t a = (uint16_t)(addr[0] | (addr[1] << 8));
  uint16_t c = (uint16_t)(cmd[0]  | (cmd[1]  << 8));
  pushPdcByteLSB(t, a, 16, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, c, 16, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  t.push_back(NEC_BIT_MARK);
  s.ok = true;
  return s;
}

Signal encodeNEC42(const uint8_t addr[4], const uint8_t cmd[4]) {
  // 13-bit address + 13-bit inverted address + 8-bit cmd + 8-bit inverted cmd.
  Signal s; s.frequencyHz = 38000;
  auto& t = s.timings;
  t.push_back(NEC_HDR_MARK); t.push_back(NEC_HDR_SPACE);
  uint32_t a = (uint32_t)(addr[0] | (addr[1] << 8)) & 0x1FFF;
  uint8_t  c = cmd[0];
  pushPdcByteLSB(t, a, 13, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, (~a) & 0x1FFF, 13, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, c, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  pushPdcByteLSB(t, (uint8_t)~c, 8, NEC_BIT_MARK, NEC_ZERO, NEC_ONE);
  t.push_back(NEC_BIT_MARK);
  s.ok = true;
  return s;
}

// ---- Samsung32 (pulse-distance, LSB first, 4500/4500 header) ----
constexpr uint16_t SAM_HDR_MARK  = 4500;
constexpr uint16_t SAM_HDR_SPACE = 4500;
constexpr uint16_t SAM_BIT_MARK  = 560;
constexpr uint16_t SAM_ZERO      = 560;
constexpr uint16_t SAM_ONE       = 1690;

Signal encodeSamsung32(const uint8_t addr[4], const uint8_t cmd[4]) {
  Signal s; s.frequencyHz = 38000;
  auto& t = s.timings;
  t.push_back(SAM_HDR_MARK); t.push_back(SAM_HDR_SPACE);
  uint8_t a = addr[0], c = cmd[0];
  pushPdcByteLSB(t, a, 8, SAM_BIT_MARK, SAM_ZERO, SAM_ONE);
  pushPdcByteLSB(t, a, 8, SAM_BIT_MARK, SAM_ZERO, SAM_ONE);
  pushPdcByteLSB(t, c, 8, SAM_BIT_MARK, SAM_ZERO, SAM_ONE);
  pushPdcByteLSB(t, (uint8_t)~c, 8, SAM_BIT_MARK, SAM_ZERO, SAM_ONE);
  t.push_back(SAM_BIT_MARK);
  s.ok = true;
  return s;
}

// ---- Sony SIRC (pulse-WIDTH coded, LSB first, 40 kHz) ----
// Header 2400 mark / 600 space, then each bit: 1 => 1200 mark, 0 => 600 mark,
// every bit followed by a 600 space. Payload = command bits then address bits.
constexpr uint16_t SIRC_HDR_MARK = 2400;
constexpr uint16_t SIRC_SPACE    = 600;
constexpr uint16_t SIRC_ONE_MARK = 1200;
constexpr uint16_t SIRC_ZERO_MARK= 600;

void pushSircBits(std::vector<uint16_t>& t, uint32_t value, int nbits) {
  for (int i = 0; i < nbits; ++i) {
    bool bit = (value >> i) & 1u;
    t.push_back(bit ? SIRC_ONE_MARK : SIRC_ZERO_MARK);
    t.push_back(SIRC_SPACE);
  }
}

Signal encodeSirc(const uint8_t addr[4], const uint8_t cmd[4],
                  int cmdBits, int addrBits) {
  Signal s; s.frequencyHz = 40000;
  auto& t = s.timings;
  t.push_back(SIRC_HDR_MARK); t.push_back(SIRC_SPACE);
  uint32_t c = cmd[0] & ((1u << cmdBits) - 1);
  uint32_t a = (uint32_t)(addr[0] | (addr[1] << 8));
  pushSircBits(t, c, cmdBits);
  pushSircBits(t, a, addrBits);
  s.ok = true;
  return s;  // trailing space already present; sendRaw tolerates it
}

// ---- Philips RC5 (Manchester, 36 kHz, 889us half-bit) ----
// Bit = 0 -> mark then space; bit = 1 -> space then mark (IEC 'logic one').
// We emit a merged raw stream (consecutive same-level halves combine).
constexpr uint16_t RC5_HALF = 889;

void pushManchesterMSB(std::vector<uint8_t>& levels, uint32_t value, int nbits) {
  // Build a level list (1 = mark, 0 = space) of half-bit cells, MSB first.
  // RC5: a '1' is space->mark, a '0' is mark->space.
  for (int i = nbits - 1; i >= 0; --i) {
    bool bit = (value >> i) & 1u;
    if (bit) { levels.push_back(0); levels.push_back(1); }
    else     { levels.push_back(1); levels.push_back(0); }
  }
}

Signal encodeRC5(const uint8_t addr[4], const uint8_t cmd[4], bool extended) {
  Signal s; s.frequencyHz = 36000;
  uint8_t a = addr[0] & 0x1F;
  uint8_t c = cmd[0] & (extended ? 0x3F : 0x3F);
  // Frame: S1 S2 Toggle A4..A0 C5..C0. S1=1. For RC5X, S2 is the inverted
  // 7th command bit; for classic RC5 S2=1. Toggle fixed to 0 (static remote).
  uint32_t frame = 0;
  int bits = 14;
  uint32_t s1 = 1;
  uint32_t s2 = extended ? ((~(c >> 6)) & 1) : 1;
  uint32_t toggle = 0;
  frame = (s1 << 13) | (s2 << 12) | (toggle << 11) |
          ((uint32_t)a << 6) | (c & 0x3F);
  std::vector<uint8_t> levels;
  pushManchesterMSB(levels, frame, bits);
  // Collapse half-bit level list into mark/space durations.
  auto& t = s.timings;
  size_t i = 0;
  // RC5 starts with a mark (S1=1 => space,mark). The very first level is a
  // space of value 0; a leading space can't start a sendRaw frame, so drop a
  // single leading space cell and begin at the first mark.
  while (i < levels.size() && levels[i] == 0) i++;
  bool curLevel = 1; // we are now at a mark
  uint32_t run = 0;
  for (; i < levels.size(); ++i) {
    if (levels[i] == curLevel) { run += RC5_HALF; }
    else { t.push_back((uint16_t)run); curLevel = levels[i]; run = RC5_HALF; }
  }
  if (run) t.push_back((uint16_t)run);
  s.ok = true;
  return s;
}

} // anonymous namespace

Signal encode(Proto p, const uint8_t addr[4], const uint8_t cmd[4]) {
  switch (p) {
    case Proto::NEC:       return encodeNEC(addr, cmd);
    case Proto::NECext:    return encodeNECext(addr, cmd);
    case Proto::NEC42:     return encodeNEC42(addr, cmd);
    case Proto::Samsung32: return encodeSamsung32(addr, cmd);
    case Proto::SIRC:      return encodeSirc(addr, cmd, 7, 5);
    case Proto::SIRC15:    return encodeSirc(addr, cmd, 7, 8);
    case Proto::SIRC20:    return encodeSirc(addr, cmd, 7, 13);
    case Proto::RC5:       return encodeRC5(addr, cmd, false);
    case Proto::RC5X:      return encodeRC5(addr, cmd, true);
    default: { Signal s; s.ok = false; return s; }
  }
}

} // namespace IrEnc
#endif // IR_ENCODERS_IMPL
