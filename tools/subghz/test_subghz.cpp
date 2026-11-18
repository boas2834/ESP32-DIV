// Host unit tests for SubGhz:: fixed-code encoders/decoders.
// Build & run:  g++ -std=c++17 -I../../ESP32-DIV test_subghz.cpp -o test_subghz && ./test_subghz
//
// These run on the dev host (no ESP32 / CC1101 needed). They exercise the pure
// waveform logic: encode a known key -> get the OOK waveform -> decode it ->
// expect the original protocol + key back (round trip), plus jitter tolerance.
#define SUBGHZ_DECODERS_IMPL
#include "subghz_decoders.h"

#include <cstdio>
#include <cstdint>
#include <random>
#include <string>

using namespace SubGhz;

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } \
    else         { printf("  ok:   %s\n", msg); } } while (0)

// Add +/- jitter to every duration to emulate a noisy real-world capture.
static std::vector<LevelDuration> jitter(std::vector<LevelDuration> w, int pct, uint32_t seed) {
    std::mt19937 rng(seed);
    for (auto& ld : w) {
        int span = (int)(ld.duration * pct / 100);
        if (span < 1) continue;
        std::uniform_int_distribution<int> d(-span, span);
        long v = (long)ld.duration + d(rng);
        ld.duration = (uint32_t)(v < 1 ? 1 : v);
    }
    return w;
}

static void roundTrip(const char* label, std::vector<LevelDuration> wave,
                      Proto wantProto, uint8_t wantBits, uint64_t wantData) {
    Decoded d = decode(wave);
    std::string m = std::string(label) + " clean";
    bool good = d.ok && d.proto == wantProto && d.bits == wantBits && d.data == wantData;
    if (!good)
        printf("    [%s] ok=%d proto=%s bits=%u data=%llx (want proto=%s bits=%u data=%llx)\n",
               label, d.ok, protoName(d.proto), d.bits, (unsigned long long)d.data,
               protoName(wantProto), wantBits, (unsigned long long)wantData);
    CHECK(good, m.c_str());

    // Same signal, 8% jitter on each edge, several seeds.
    for (uint32_t s = 1; s <= 3; ++s) {
        Decoded dj = decode(jitter(wave, 8, s * 101));
        std::string mj = std::string(label) + " +jitter seed" + std::to_string(s);
        bool gj = dj.ok && dj.proto == wantProto && dj.bits == wantBits && dj.data == wantData;
        CHECK(gj, mj.c_str());
    }
}

int main() {
    printf("Princeton\n");
    roundTrip("Princeton 0xA5B3C4", encodePrinceton(0xA5B3C4, 24),
              Proto::Princeton, 24, 0xA5B3C4);

    printf("CAME 12\n");
    roundTrip("CAME12 0x334", encodeCame(0x334, 12), Proto::CAME, 12, 0x334);

    printf("CAME 24\n");
    roundTrip("CAME24 0x0ABCDE", encodeCame(0x0ABCDE, 24), Proto::CAME24, 24, 0x0ABCDE);

    printf("Nice FLO 12\n");
    roundTrip("NiceFlo12 0x5A5", encodeNiceFlo(0x5A5, 12), Proto::NiceFlo, 12, 0x5A5);

    printf("Nice FLO 24\n");
    roundTrip("NiceFlo24 0x123456", encodeNiceFlo(0x123456, 24), Proto::NiceFlo24, 24, 0x123456);

    printf("Holtek HT12 (valid header 0x5...)\n");
    roundTrip("Holtek 0x5123456789", encodeHoltek(0x5123456789ULL, 40),
              Proto::Holtek, 40, 0x5123456789ULL);

    printf("Holtek with wrong header must NOT decode as Holtek\n");
    {
        Decoded d = decode(encodeHoltek(0x1123456789ULL, 40)); // top nibble 1, not 5
        CHECK(!(d.ok && d.proto == Proto::Holtek), "bad-header Holtek rejected");
    }

    printf("Re-encode (TX round trip) of a decoded signal\n");
    {
        Decoded d = decode(encodeCame(0x0ABCDE, 24));
        auto re = encode(d);
        Decoded d2 = decode(re);
        CHECK(d2.ok && d2.proto == Proto::CAME24 && d2.data == 0x0ABCDE, "CAME24 re-encode stable");
    }

    printf("Noise-only input decodes to nothing\n");
    {
        std::vector<LevelDuration> noise;
        std::mt19937 rng(7);
        std::uniform_int_distribution<uint32_t> d(80, 5000);
        for (int i = 0; i < 200; ++i) noise.push_back({(i & 1) == 0, d(rng)});
        Decoded r = decode(noise);
        CHECK(!r.ok, "random noise not falsely decoded");
    }

    printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
           g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
