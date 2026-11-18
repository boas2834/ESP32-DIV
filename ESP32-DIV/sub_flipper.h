// sub_flipper.h — pure, hardware-independent parser for Flipper Zero ".sub"
// Sub-GHz files, plus a builder that turns a parsed file into the OOK waveform
// the CC1101 transmit layer plays back.
//
// Reimplemented from the public .sub text format (key: value lines, RAW_Data
// as signed microsecond runs). Fixed-code protocols are re-synthesised through
// the SubGhz:: encoders (subghz_decoders.h); RAW files are replayed as-is.
// Rolling-code protocols are recognised but never re-encoded for transmit.
//
// Header-only, stb-style: define SUB_FLIPPER_IMPL in exactly one TU
// (sub_flipper.cpp) to emit the implementation.
#ifndef SUB_FLIPPER_H
#define SUB_FLIPPER_H

#include <stdint.h>
#include <string>
#include <vector>
#include <functional>
#include "subghz_decoders.h"

namespace SubFlip {

enum class Mod : uint8_t { Unknown, OOK, FSK };

struct SubFile {
    uint32_t frequencyHz = 0;
    Mod mod = Mod::Unknown;
    bool raw = false;                 // RAW_Data file vs a protocol file
    std::vector<int32_t> rawData;     // signed us runs: + = carrier on, - = off
    std::string protocol;             // protocol name as written in the file
    uint8_t  bits = 0;
    uint64_t key = 0;                 // meaningful in the low `bits` bits
    bool rolling = false;             // recognised rolling code (not re-sendable)
    bool ok = false;
    std::string error;
};

// Pulls successive lines (without the newline). Returns false at EOF.
using LineReader = std::function<bool(std::string&)>;

SubFile parse(LineReader next);
SubFile parseString(const std::string& text);

// Map the file's protocol name onto a SubGhz:: fixed-code protocol, or
// Proto::Unknown (RAW, rolling, or unsupported).
SubGhz::Proto protoFor(const SubFile& f);

// True when this file can actually be transmitted: OOK, and either RAW or an
// encodable fixed-code protocol, and not a rolling code.
bool isSendable(const SubFile& f);

// Build the OOK waveform to transmit (empty if not sendable).
std::vector<SubGhz::LevelDuration> toWaveform(const SubFile& f);

} // namespace SubFlip

#ifdef SUB_FLIPPER_IMPL
#include <cctype>
#include <cstdlib>

namespace SubFlip {

static inline void trimStr(std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (unsigned char)s[a] <= ' ') ++a;
    while (b > a && (unsigned char)s[b - 1] <= ' ') --b;
    s = s.substr(a, b - a);
}
static inline std::string lowerStr(std::string s) {
    for (auto& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}
static inline bool containsCI(const std::string& hay, const char* needle) {
    return lowerStr(hay).find(needle) != std::string::npos;
}

// Rolling-code protocol names (display-only; never re-encoded for transmit).
static bool isRollingName(const std::string& p) {
    static const char* kRolling[] = {
        "keeloq", "nice flor", "flor-s", "flors", "somfy", "star line", "starline",
        "came atomo", "atomo", "hormann", "faac slh", "alutech", "security+",
        "securityplus", "kia", "an-motors", "an motors"};
    std::string l = lowerStr(p);
    for (const char* r : kRolling) if (l.find(r) != std::string::npos) return true;
    return false;
}

SubGhz::Proto protoFor(const SubFile& f) {
    if (f.raw || f.rolling) return SubGhz::Proto::Unknown;
    std::string p = lowerStr(f.protocol);
    if (p.find("princeton") != std::string::npos) return SubGhz::Proto::Princeton;
    if (p.find("holtek")    != std::string::npos) return SubGhz::Proto::Holtek;
    if (p.find("nice")      != std::string::npos)  // Nice FLO (12/24)
        return f.bits >= 24 ? SubGhz::Proto::NiceFlo24 : SubGhz::Proto::NiceFlo;
    if (p.find("came")      != std::string::npos)  // CAME (12/24)
        return f.bits >= 24 ? SubGhz::Proto::CAME24 : SubGhz::Proto::CAME;
    return SubGhz::Proto::Unknown;
}

bool isSendable(const SubFile& f) {
    if (!f.ok || f.mod != Mod::OOK || f.rolling) return false;
    if (f.raw) return f.rawData.size() >= 2;
    return protoFor(f) != SubGhz::Proto::Unknown;
}

static void parseKeyHex(const std::string& val, SubFile& f) {
    uint64_t k = 0; int nbytes = 0;
    size_t i = 0;
    while (i < val.size()) {
        while (i < val.size() && !std::isxdigit((unsigned char)val[i])) ++i;
        if (i >= val.size()) break;
        std::string tok;
        while (i < val.size() && std::isxdigit((unsigned char)val[i])) tok += val[i++];
        k = (k << 8) | (uint64_t)(std::strtoul(tok.c_str(), nullptr, 16) & 0xFF);
        ++nbytes;
    }
    f.key = k;
}

SubFile parse(LineReader next) {
    SubFile f;
    std::string line;
    bool sawFiletype = false;
    while (next(line)) {
        trimStr(line);
        if (line.empty() || line[0] == '#') continue;
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        trimStr(key); trimStr(val);
        std::string lk = lowerStr(key);

        if (lk == "filetype") { sawFiletype = containsCI(val, "flipper") && containsCI(val, "subghz"); }
        else if (lk == "frequency") { f.frequencyHz = (uint32_t)std::strtoul(val.c_str(), nullptr, 10); }
        else if (lk == "preset") {
            if (containsCI(val, "ook")) f.mod = Mod::OOK;
            else if (containsCI(val, "fsk")) f.mod = Mod::FSK;
        }
        else if (lk == "protocol") {
            f.protocol = val;
            if (lowerStr(val) == "raw") f.raw = true;
            if (isRollingName(val)) f.rolling = true;
        }
        else if (lk == "bit") { f.bits = (uint8_t)std::strtoul(val.c_str(), nullptr, 10); }
        else if (lk == "key") { parseKeyHex(val, f); }
        else if (lk == "raw_data") {
            f.raw = true;
            const char* s = val.c_str();
            char* end = nullptr;
            while (*s) {
                long v = std::strtol(s, &end, 10);
                if (end == s) break;
                if (v != 0) f.rawData.push_back((int32_t)v);
                s = end;
            }
        }
    }
    if (f.mod == Mod::Unknown && (f.raw || !f.protocol.empty())) f.mod = Mod::OOK; // tolerate missing preset
    f.ok = sawFiletype && f.frequencyHz > 0 && (f.raw ? !f.rawData.empty() : !f.protocol.empty());
    if (!f.ok && f.error.empty()) f.error = "incomplete .sub";
    return f;
}

SubFile parseString(const std::string& text) {
    size_t pos = 0;
    return parse([&text, &pos](std::string& out) -> bool {
        if (pos >= text.size()) return false;
        size_t nl = text.find('\n', pos);
        if (nl == std::string::npos) { out = text.substr(pos); pos = text.size(); }
        else { out = text.substr(pos, nl - pos); pos = nl + 1; }
        return true;
    });
}

std::vector<SubGhz::LevelDuration> toWaveform(const SubFile& f) {
    std::vector<SubGhz::LevelDuration> w;
    if (!isSendable(f)) return w;
    if (f.raw) {
        w.reserve(f.rawData.size());
        for (int32_t d : f.rawData) {
            if (d == 0) continue;
            w.push_back({ d > 0, (uint32_t)(d > 0 ? d : -d) });
        }
        return w;
    }
    switch (protoFor(f)) {
        case SubGhz::Proto::Princeton: return SubGhz::encodePrinceton(f.key, f.bits ? f.bits : 24);
        case SubGhz::Proto::CAME:
        case SubGhz::Proto::CAME24:    return SubGhz::encodeCame(f.key, f.bits ? f.bits : 12);
        case SubGhz::Proto::NiceFlo:
        case SubGhz::Proto::NiceFlo24: return SubGhz::encodeNiceFlo(f.key, f.bits ? f.bits : 12);
        case SubGhz::Proto::Holtek:    return SubGhz::encodeHoltek(f.key, f.bits ? f.bits : 40);
        default:                       return w;
    }
}

} // namespace SubFlip
#endif // SUB_FLIPPER_IMPL

#endif // SUB_FLIPPER_H
