// Host unit tests for the Flipper .sub parser + waveform builder (SubFlip).
// Build & run:
//   g++ -std=c++17 -I../../ESP32-DIV test_sub.cpp -o test_sub && ./test_sub
#define SUBGHZ_DECODERS_IMPL
#define SUB_FLIPPER_IMPL
#include "sub_flipper.h"     // pulls in subghz_decoders.h

#include <cstdio>
#include <string>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } \
    else         { printf("  ok:   %s\n", msg); } } while (0)

using namespace SubFlip;

int main() {
    printf("RAW .sub\n");
    {
        std::string s =
            "Filetype: Flipper SubGhz RAW File\n"
            "Version: 1\n"
            "Frequency: 433920000\n"
            "Preset: FuriHalSubGhzPresetOok650Async\n"
            "Protocol: RAW\n"
            "RAW_Data: 300 -300 900 -900\n"
            "RAW_Data: 300 -9000\n";
        SubFile f = parseString(s);
        CHECK(f.ok && f.raw && f.mod == Mod::OOK, "RAW parsed, OOK");
        CHECK(f.frequencyHz == 433920000, "frequency");
        CHECK(f.rawData.size() == 6, "all RAW_Data lines appended");
        CHECK(isSendable(f), "RAW sendable");
        auto w = toWaveform(f);
        bool shape = w.size() == 6 && w[0].level && w[0].duration == 300 &&
                     !w[1].level && w[1].duration == 300 && w[2].level && w[2].duration == 900 &&
                     !w[5].level && w[5].duration == 9000;
        CHECK(shape, "RAW waveform levels/durations");
    }

    printf("Princeton protocol .sub\n");
    {
        std::string s =
            "Filetype: Flipper SubGhz Key File\n"
            "Frequency: 433920000\n"
            "Preset: FuriHalSubGhzPresetOok650Async\n"
            "Protocol: Princeton\n"
            "Bit: 24\n"
            "Key: 00 00 00 00 00 A5 B3 C4\n";
        SubFile f = parseString(s);
        CHECK(f.ok && !f.raw && f.key == 0xA5B3C4 && f.bits == 24, "Princeton parsed");
        CHECK(isSendable(f), "Princeton sendable");
        auto w = toWaveform(f);
        SubGhz::Decoded d = SubGhz::decode(w);
        CHECK(d.ok && d.proto == SubGhz::Proto::Princeton && d.data == 0xA5B3C4,
              "Princeton waveform decodes back");
    }

    printf("CAME 12 protocol .sub\n");
    {
        std::string s =
            "Filetype: Flipper SubGhz Key File\n"
            "Frequency: 433920000\n"
            "Preset: FuriHalSubGhzPresetOok650Async\n"
            "Protocol: CAME\n"
            "Bit: 12\n"
            "Key: 00 00 00 00 00 00 03 34\n";
        SubFile f = parseString(s);
        auto w = toWaveform(f);
        SubGhz::Decoded d = SubGhz::decode(w);
        CHECK(d.ok && d.proto == SubGhz::Proto::CAME && d.data == 0x334, "CAME12 round trip");
    }

    printf("FSK preset is not sendable (OOK TX only)\n");
    {
        std::string s =
            "Filetype: Flipper SubGhz RAW File\n"
            "Frequency: 868350000\n"
            "Preset: FuriHalSubGhzPreset2FSKDev238Async\n"
            "Protocol: RAW\n"
            "RAW_Data: 100 -100 100 -100\n";
        SubFile f = parseString(s);
        CHECK(f.mod == Mod::FSK, "FSK preset detected");
        CHECK(!isSendable(f), "FSK not sendable");
    }

    printf("Rolling code recognised, not sendable\n");
    {
        std::string s =
            "Filetype: Flipper SubGhz Key File\n"
            "Frequency: 433920000\n"
            "Preset: FuriHalSubGhzPresetOok650Async\n"
            "Protocol: KeeLoq\n"
            "Bit: 64\n"
            "Key: 11 22 33 44 55 66 77 88\n";
        SubFile f = parseString(s);
        CHECK(f.rolling, "KeeLoq flagged rolling");
        CHECK(!isSendable(f), "rolling not sendable");
    }

    printf("Missing Filetype -> not ok\n");
    {
        std::string s = "Frequency: 433920000\nProtocol: RAW\nRAW_Data: 1 -1\n";
        SubFile f = parseString(s);
        CHECK(!f.ok, "no Filetype rejected");
    }

    printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
           g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
