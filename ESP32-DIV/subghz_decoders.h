// subghz_decoders.h — pure, hardware-independent Sub-GHz fixed-code
// encoders/decoders for the ESP32-DIV port.
//
// Reimplemented from public protocol timing facts (PT2262/Princeton, CAME,
// Nice FLO, Holtek HT12). Only numeric timing constants (te_short/te_long/…)
// are reused — these are facts about the wire format, not copied code.
//
// Header-only, stb-style: the declarations are always visible; the
// implementations compile only in the single translation unit that defines
// SUBGHZ_DECODERS_IMPL before including this header (see subghz_decoders.cpp).
//
// A "signal" here is the over-the-air OOK waveform expressed as a list of
// LevelDuration pairs (level=true -> carrier/HIGH, false -> gap/LOW), which is
// exactly what the CC1101 capture layer produces and what the TX layer plays
// back. Encoders build that waveform; the unified decode() recognises it.
#ifndef SUBGHZ_DECODERS_H
#define SUBGHZ_DECODERS_H

#include <stdint.h>
#include <vector>

namespace SubGhz {

struct LevelDuration {
    bool level;         // true = HIGH (carrier on), false = LOW (gap)
    uint32_t duration;  // microseconds
};

enum class Proto : uint8_t {
    Unknown = 0,
    Princeton,   // PT2262/EV1527 style, 24 bit
    CAME,        // 12 bit
    CAME24,      // 24 bit
    NiceFlo,     // 12 bit
    NiceFlo24,   // 24 bit
    Holtek,      // HT12, 40 bit
};

struct Decoded {
    Proto proto = Proto::Unknown;
    const char* name = "Unknown";
    uint8_t bits = 0;        // number of data bits recovered
    uint64_t data = 0;       // raw key, MSB-first in the low `bits` bits
    bool rolling = false;    // true -> rolling code: shown but not re-sendable
    bool ok = false;
};

const char* protoName(Proto p);

// --- Encoders: build the OOK waveform for a given key ---------------------
std::vector<LevelDuration> encodePrinceton(uint64_t data, uint8_t bits = 24);
std::vector<LevelDuration> encodeCame(uint64_t data, uint8_t bits = 12);
std::vector<LevelDuration> encodeNiceFlo(uint64_t data, uint8_t bits = 12);
std::vector<LevelDuration> encodeHoltek(uint64_t data, uint8_t bits = 40);

// Re-encode a decoded fixed-code signal for retransmission. Returns an empty
// vector for rolling codes / unknown protocols (not safely replayable).
std::vector<LevelDuration> encode(const Decoded& d);

// --- Decoder: try every known fixed-code protocol -------------------------
Decoded decode(const std::vector<LevelDuration>& wave);

} // namespace SubGhz

#ifdef SUBGHZ_DECODERS_IMPL
namespace SubGhz {

// |value - ref| <= delta
static inline bool tMatch(uint32_t value, uint32_t ref, uint32_t delta) {
    return (value + delta >= ref) && (value <= ref + delta);
}

static inline bool bitRead64(uint64_t v, uint8_t i) { return (v >> i) & 1ULL; }

const char* protoName(Proto p) {
    switch (p) {
        case Proto::Princeton: return "Princeton";
        case Proto::CAME:      return "CAME 12";
        case Proto::CAME24:    return "CAME 24";
        case Proto::NiceFlo:   return "Nice FLO 12";
        case Proto::NiceFlo24: return "Nice FLO 24";
        case Proto::Holtek:    return "Holtek HT12";
        default:               return "Unknown";
    }
}

// ----------------------------- timing tables ------------------------------
// Princeton: short te, long = 3*te, trailing guard = 30*te (LOW).
static const uint32_t PT_TE    = 390;
static const uint32_t PT_DELTA = 200;
static const uint32_t PT_GUARD = 30;   // * te
// CAME
static const uint32_t CAME_SHORT = 320, CAME_LONG = 640, CAME_DELTA = 150;
static const uint32_t CAME_HDR12 = 36, CAME_HDR24 = 36; // * short (LOW header)
// Nice FLO
static const uint32_t FLO_SHORT = 700, FLO_LONG = 1400, FLO_DELTA = 250;
static const uint32_t FLO_HDR = 36;    // * short (LOW header)
// Holtek HT12
static const uint32_t HT_SHORT = 430, HT_LONG = 870, HT_DELTA = 110;
static const uint32_t HT_HDR = 36;     // * short (LOW header)
static const uint64_t HOLTEK_HEADER = 0x5000000000ULL;
static const uint64_t HOLTEK_HEADER_MASK = 0xF000000000ULL;

// ------------------------------- encoders ---------------------------------
// Princeton: per bit HIGH-first. bit1 = HIGH 3te + LOW te; bit0 = HIGH te +
// LOW 3te. Then a stop HIGH te and a long LOW guard.
std::vector<LevelDuration> encodePrinceton(uint64_t data, uint8_t bits) {
    std::vector<LevelDuration> w;
    w.reserve(bits * 2 + 2);
    for (int i = bits; i > 0; --i) {
        if (bitRead64(data, i - 1)) {
            w.push_back({true, PT_TE * 3});
            w.push_back({false, PT_TE});
        } else {
            w.push_back({true, PT_TE});
            w.push_back({false, PT_TE * 3});
        }
    }
    w.push_back({true, PT_TE});                  // stop bit
    w.push_back({false, PT_TE * PT_GUARD});      // guard
    return w;
}

// CAME / Nice FLO / Holtek share a shape: LOW header, HIGH start bit, then per
// bit LOW-first. bit1 = LOW long + HIGH short; bit0 = LOW short + HIGH long.
static std::vector<LevelDuration> encodeLowFirst(
    uint64_t data, uint8_t bits, uint32_t sh, uint32_t lo, uint32_t hdrMul) {
    std::vector<LevelDuration> w;
    w.reserve(bits * 2 + 2);
    w.push_back({false, sh * hdrMul});           // header (LOW)
    w.push_back({true, sh});                      // start bit (HIGH)
    for (int i = bits; i > 0; --i) {
        if (bitRead64(data, i - 1)) {
            w.push_back({false, lo});
            w.push_back({true, sh});
        } else {
            w.push_back({false, sh});
            w.push_back({true, lo});
        }
    }
    return w;
}

std::vector<LevelDuration> encodeCame(uint64_t data, uint8_t bits) {
    return encodeLowFirst(data, bits, CAME_SHORT, CAME_LONG, CAME_HDR12);
}
std::vector<LevelDuration> encodeNiceFlo(uint64_t data, uint8_t bits) {
    return encodeLowFirst(data, bits, FLO_SHORT, FLO_LONG, FLO_HDR);
}
std::vector<LevelDuration> encodeHoltek(uint64_t data, uint8_t bits) {
    return encodeLowFirst(data, bits, HT_SHORT, HT_LONG, HT_HDR);
}

std::vector<LevelDuration> encode(const Decoded& d) {
    if (d.rolling) return {};
    switch (d.proto) {
        case Proto::Princeton: return encodePrinceton(d.data, d.bits);
        case Proto::CAME:
        case Proto::CAME24:    return encodeCame(d.data, d.bits);
        case Proto::NiceFlo:
        case Proto::NiceFlo24: return encodeNiceFlo(d.data, d.bits);
        case Proto::Holtek:    return encodeHoltek(d.data, d.bits);
        default:               return {};
    }
}

// ------------------------------- decoders ---------------------------------
// Princeton: read HIGH/LOW bit pairs (bit1 = HIGH 3te + LOW te; bit0 = HIGH te
// + LOW 3te). A frame ends with a stop HIGH te followed by a long guard LOW; at
// that terminator we accept if exactly 24 data bits were collected. This copes
// with a frame that precedes its guard and with repeated frames.
static Decoded decodePrinceton(const std::vector<LevelDuration>& w) {
    Decoded out;
    const uint32_t longLow = PT_TE * 6; // clearly longer than 3*te -> guard
    uint64_t data = 0; uint8_t bits = 0;
    size_t n = w.size();
    for (size_t i = 0; i + 1 < n; ) {
        const LevelDuration& hi = w[i];
        const LevelDuration& lo = w[i + 1];
        if (!hi.level || lo.level) { data = 0; bits = 0; ++i; continue; }
        if (tMatch(hi.duration, PT_TE * 3, PT_DELTA) &&
            tMatch(lo.duration, PT_TE, PT_DELTA)) {
            data = (data << 1) | 1ULL; if (++bits > 32) { data = 0; bits = 0; }
            i += 2; continue;
        }
        if (tMatch(hi.duration, PT_TE, PT_DELTA) &&
            tMatch(lo.duration, PT_TE * 3, PT_DELTA)) {
            data = (data << 1); if (++bits > 32) { data = 0; bits = 0; }
            i += 2; continue;
        }
        // Terminator: stop bit (HIGH ~te) + guard (LOW, long). Accept here.
        if (lo.duration >= longLow && bits == 24) {
            out.proto = Proto::Princeton; out.name = protoName(Proto::Princeton);
            out.bits = bits; out.data = data; out.ok = true;
            return out;
        }
        data = 0; bits = 0; ++i; // unrecognised -> resync
    }
    return out;
}

// CAME/Nice FLO/Holtek: sync on the LOW header, skip the HIGH start bit, then
// read LOW/HIGH pairs. bit1 = LOW long + HIGH short; bit0 = LOW short+HIGH long.
static Decoded decodeLowFirst(const std::vector<LevelDuration>& w, uint32_t sh,
                              uint32_t lo, uint32_t delta, uint32_t hdrMul,
                              uint8_t wantBits, Proto proto) {
    Decoded out;
    const uint32_t header = sh * hdrMul;
    const uint32_t hdrDelta = sh * 6; // generous — header is much longer than a bit
    size_t n = w.size();
    for (size_t start = 0; start + 1 < n; ++start) {
        if (w[start].level) continue;
        if (!tMatch(w[start].duration, header, hdrDelta)) continue;
        if (!w[start + 1].level || !tMatch(w[start + 1].duration, sh, delta))
            continue;                                      // need HIGH start bit
        uint64_t data = 0; uint8_t bits = 0; size_t i = start + 2;
        while (i + 1 < n) {
            const LevelDuration& a = w[i];     // LOW
            const LevelDuration& b = w[i + 1]; // HIGH
            if (a.level || !b.level) break;
            bool bit;
            if (tMatch(a.duration, lo, delta) && tMatch(b.duration, sh, delta)) {
                bit = true;
            } else if (tMatch(a.duration, sh, delta) && tMatch(b.duration, lo, delta)) {
                bit = false;
            } else {
                break;
            }
            data = (data << 1) | (bit ? 1 : 0);
            ++bits;
            if (bits >= wantBits) { i += 2; break; }
            i += 2;
        }
        if (bits == wantBits) {
            if (proto == Proto::Holtek &&
                (data & HOLTEK_HEADER_MASK) != HOLTEK_HEADER)
                continue; // not a valid Holtek frame
            out.proto = proto; out.name = protoName(proto);
            out.bits = bits; out.data = data; out.ok = true;
            return out;
        }
    }
    return out;
}

Decoded decode(const std::vector<LevelDuration>& wave) {
    Decoded d;
    // Longest/most specific frames first to avoid a short prefix matching.
    d = decodeLowFirst(wave, HT_SHORT, HT_LONG, HT_DELTA, HT_HDR, 40, Proto::Holtek);
    if (d.ok) return d;
    d = decodeLowFirst(wave, CAME_SHORT, CAME_LONG, CAME_DELTA, CAME_HDR24, 24, Proto::CAME24);
    if (d.ok) return d;
    d = decodeLowFirst(wave, CAME_SHORT, CAME_LONG, CAME_DELTA, CAME_HDR12, 12, Proto::CAME);
    if (d.ok) return d;
    d = decodeLowFirst(wave, FLO_SHORT, FLO_LONG, FLO_DELTA, FLO_HDR, 24, Proto::NiceFlo24);
    if (d.ok) return d;
    d = decodeLowFirst(wave, FLO_SHORT, FLO_LONG, FLO_DELTA, FLO_HDR, 12, Proto::NiceFlo);
    if (d.ok) return d;
    d = decodePrinceton(wave);
    if (d.ok) return d;
    return Decoded{};
}

} // namespace SubGhz
#endif // SUBGHZ_DECODERS_IMPL

#endif // SUBGHZ_DECODERS_H
