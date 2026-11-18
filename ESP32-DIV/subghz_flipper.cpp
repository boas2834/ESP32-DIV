// subghz_flipper.cpp — "Flipper .sub" Sub-GHz feature for ESP32-DIV.
//
// Browses Flipper Zero .sub files on the SD card (/subghz, with subdirectories),
// shows the parsed signal, and transmits it on the CC1101 in OOK. RAW captures
// are replayed from their microsecond runs; fixed-code protocols are
// re-synthesised by the tested SubGhz:: encoders. Rolling codes are display-only.
//
// UI mirrors the IR Flipper browser: hold Up/Down to auto-scroll, tap a row to
// select then tap again (or press centre) to open/send, Left goes up a folder
// or exits at the root.

#include <Arduino.h>
#include <algorithm>
#include <vector>
#include <string>

#include "config.h"        // SD, SPI, ELECHOUSE_cc1101, RCSwitch, extern tft
#include "shared.h"        // pins, colours, feature_active / feature_exit_requested
#include "utils.h"         // nav/touch helpers, SD helpers, drawStatusBar
#include "Touchscreen.h"   // readTouchXY
#include "subghz_flipper.h"
#include "sub_flipper.h"   // SubFlip:: parser + waveform builder (decls only)

namespace SubFlipper {

// ---- directory + UI layout -------------------------------------------------
static const char* kRootDir = "/subghz";

static const int16_t kTitleY   = 24;
static const int16_t kListTop  = 40;
static const int16_t kLineH    = 20;
static const int16_t kListX    = 8;

enum class Screen : uint8_t { Browse, Detail };

struct Child { String name; bool isDir; };

static bool               s_active = false;
static Screen             s_screen = Screen::Browse;
static String             s_dir;                 // relative path under kRootDir, "" = root
static std::vector<Child> s_kids;
static int                s_sel = 0;
static int                s_first = 0;           // first visible row
static String             s_status;

// Detail state
static String      s_filePath;                   // full SD path of opened .sub
static SubFlip::SubFile s_parsed;
static bool        s_parsedOk = false;

// auto-repeat scroll
static bool     s_repActive = false;
static int      s_repDir = 0;
static uint32_t s_repNext = 0;

// ---- helpers ---------------------------------------------------------------
static int16_t listBottomY() {
    int16_t b = featureHasTouchNavBar() ? touchNavContentBottomY() : (int16_t)tft.height();
    return b - 2;
}
static int visibleRows() {
    int rows = (listBottomY() - kListTop) / kLineH;
    if (rows < 1) rows = 1;
    return rows;
}

static String rootPath() { return String(kRootDir); }
static String curDirPath() {
    String p = rootPath();
    if (s_dir.length()) { p += "/"; p += s_dir; }
    return p;
}

static String basenameOf(const char* n) {
    String s(n);
    int slash = s.lastIndexOf('/');
    return (slash >= 0) ? s.substring(slash + 1) : s;
}

static bool endsWithCI(const String& s, const char* suf) {
    int n = strlen(suf);
    if (s.length() < n) return false;
    for (int i = 0; i < n; ++i) {
        char a = s[s.length() - n + i]; char b = suf[i];
        if (a >= 'A' && a <= 'Z') a += 32;
        if (b >= 'A' && b <= 'Z') b += 32;
        if (a != b) return false;
    }
    return true;
}

static void scanDir() {
    s_kids.clear();
    if (!isSDCardAvailable()) { restoreSdAfterSharedSpi(); }
    String path = curDirPath();
    File d = SD.open(path.c_str());
    if (d) {
        for (;;) {
            File f = d.openNextFile();
            if (!f) break;
            String name = basenameOf(f.name());
            bool dir = f.isDirectory();
            f.close();
            if (!name.length() || name[0] == '.') continue;
            if (dir) { s_kids.push_back({ name, true }); }
            else if (endsWithCI(name, ".sub")) { s_kids.push_back({ name, false }); }
        }
        d.close();
    }
    // directories first, then files, each alphabetical (case-insensitive-ish)
    std::sort(s_kids.begin(), s_kids.end(), [](const Child& a, const Child& b) {
        if (a.isDir != b.isDir) return a.isDir > b.isDir;
        String la = a.name; la.toLowerCase();
        String lb = b.name; lb.toLowerCase();
        return la < lb;
    });
    if (s_sel >= (int)s_kids.size()) s_sel = (int)s_kids.size() - 1;
    if (s_sel < 0) s_sel = 0;
    s_first = 0;
}

static bool readFileToString(const String& path, std::string& out, size_t cap = 96 * 1024) {
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) return false;
    out.clear();
    uint8_t buf[256];
    while (f.available() && out.size() < cap) {
        int n = f.read(buf, sizeof(buf));
        if (n <= 0) break;
        out.append((const char*)buf, (size_t)n);
    }
    f.close();
    return true;
}

// ---- transmit --------------------------------------------------------------
static void sfUsDelay(uint32_t us) {
    while (us > 16000) { delayMicroseconds(16000); us -= 16000; }
    if (us) delayMicroseconds(us);
}

static void sfTxWaveform(uint32_t freqHz, const std::vector<SubGhz::LevelDuration>& w, int repeats) {
    if (w.empty() || freqHz == 0) return;
    holdSdInactiveOnSharedSpi();
    reclaimSharedSpiBus();
#if defined(SD_CS)
    pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
#endif
#if defined(CC1101_CS)
    pinMode(CC1101_CS, OUTPUT); digitalWrite(CC1101_CS, HIGH);
#endif
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
    ELECHOUSE_cc1101.setCCMode(0);
    ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
    ELECHOUSE_cc1101.setPA(12);
    pinMode(SUBGHZ_TX_PIN, OUTPUT);
    digitalWrite(SUBGHZ_TX_PIN, LOW);
    ELECHOUSE_cc1101.SetTx();

    for (int r = 0; r < repeats; ++r) {
        for (const SubGhz::LevelDuration& ld : w) {
            digitalWrite(SUBGHZ_TX_PIN, ld.level ? HIGH : LOW);
            sfUsDelay(ld.duration);
        }
        digitalWrite(SUBGHZ_TX_PIN, LOW);
        if (r + 1 < repeats) delay(12);  // inter-frame gap
    }

    digitalWrite(SUBGHZ_TX_PIN, LOW);
    ELECHOUSE_cc1101.setSidle();
    restoreSdAfterSharedSpi();
}

// ---- drawing ---------------------------------------------------------------
static void drawNavLabels() {
    if (s_screen == Screen::Browse) {
        const char* left = s_dir.length() ? "Zurueck" : "Exit";
        setTouchNavLabels(left, "Runter", "Oeffnen", "Hoch", "Oeffnen");
    } else {
        setTouchNavLabels("Zurueck", " ", "Senden", " ", "Senden");
    }
}

static void drawBrowse() {
    tft.fillScreen(FEATURE_BG);
    drawStatusBar(readBatteryVoltage(), true);

    tft.setTextSize(1);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(kListX, kTitleY);
    String title = "Flipper .sub";
    if (s_dir.length()) { title += "  /"; title += s_dir; }
    tft.print(title);
    tft.drawFastHLine(0, kTitleY + 11, tft.width(), UI_LINE);

    const int rows = visibleRows();
    if (s_sel < s_first) s_first = s_sel;
    if (s_sel >= s_first + rows) s_first = s_sel - rows + 1;

    if (s_kids.empty()) {
        tft.setTextColor(UI_DIM_TEXT, FEATURE_BG);
        tft.setCursor(kListX, kListTop + 4);
        tft.print(isSDCardAvailable() ? "Keine .sub-Dateien" : "Keine SD-Karte");
    }

    for (int i = 0; i < rows; ++i) {
        int idx = s_first + i;
        if (idx >= (int)s_kids.size()) break;
        int16_t y = kListTop + i * kLineH;
        bool seld = (idx == s_sel);
        if (seld) tft.fillRect(0, y - 2, tft.width(), kLineH, UI_LINE);
        tft.setTextColor(seld ? FEATURE_BG : UI_TEXT, seld ? UI_LINE : FEATURE_BG);
        tft.setCursor(kListX, y + 2);
        const Child& c = s_kids[idx];
        String label = c.isDir ? (String("[") + c.name + "]") : c.name;
        if (label.length() > 36) label = label.substring(0, 35) + "~";
        tft.print(label);
    }

    drawNavLabels();
}

static void drawDetail() {
    tft.fillScreen(FEATURE_BG);
    drawStatusBar(readBatteryVoltage(), true);

    tft.setTextSize(1);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(kListX, kTitleY);
    String fname = basenameOf(s_filePath.c_str());
    if (fname.length() > 36) fname = fname.substring(0, 35) + "~";
    tft.print(fname);
    tft.drawFastHLine(0, kTitleY + 11, tft.width(), UI_LINE);

    int16_t y = kListTop + 4;
    auto line = [&](const String& s, uint16_t col) {
        tft.setTextColor(col, FEATURE_BG);
        tft.setCursor(kListX, y);
        tft.print(s);
        y += kLineH;
    };

    if (!s_parsedOk || !s_parsed.ok) {
        line("Datei nicht lesbar / ungueltig", UI_WARN);
    } else {
        const SubFlip::SubFile& f = s_parsed;
        char fb[32];
        snprintf(fb, sizeof(fb), "%.3f MHz", f.frequencyHz / 1000000.0);
        line(String("Frequenz: ") + fb, UI_TEXT);
        line(String("Mod: ") + (f.mod == SubFlip::Mod::OOK ? "OOK" :
                                 f.mod == SubFlip::Mod::FSK ? "FSK (2-FSK)" : "?"), UI_TEXT);
        if (f.raw) {
            line(String("Typ: RAW  (") + (int)f.rawData.size() + " Flanken)", UI_TEXT);
        } else {
            line(String("Protokoll: ") + f.protocol.c_str(), UI_TEXT);
            char kb[40];
            snprintf(kb, sizeof(kb), "Key: %llX  (%u Bit)",
                     (unsigned long long)f.key, (unsigned)f.bits);
            line(String(kb), UI_TEXT);
        }

        bool send = SubFlip::isSendable(f);
        if (f.rolling)
            line("Rolling-Code: nur Anzeige", UI_WARN);
        else if (!send && f.mod == SubFlip::Mod::FSK)
            line("2-FSK: Senden nicht moeglich", UI_WARN);
        else if (!send)
            line("Nicht sendbar", UI_WARN);
        else
            line("Bereit zum Senden", UI_TEXT);
    }

    y += 6;
    if (s_status.length()) {
        tft.setTextColor(UI_TEXT, FEATURE_BG);
        tft.setCursor(kListX, y);
        tft.print(s_status);
    }

    drawNavLabels();
}

static void redraw() {
    if (s_screen == Screen::Browse) drawBrowse();
    else drawDetail();
}

// ---- actions ---------------------------------------------------------------
static void openSelected() {
    if (s_sel < 0 || s_sel >= (int)s_kids.size()) return;
    const Child c = s_kids[s_sel];
    if (c.isDir) {
        if (s_dir.length()) { s_dir += "/"; s_dir += c.name; }
        else s_dir = c.name;
        s_sel = 0; s_first = 0;
        scanDir();
        s_screen = Screen::Browse;
        redraw();
        return;
    }
    // open .sub file
    s_filePath = curDirPath() + "/" + c.name;
    std::string text;
    s_parsedOk = readFileToString(s_filePath, text);
    s_parsed = s_parsedOk ? SubFlip::parseString(text) : SubFlip::SubFile{};
    s_status = "";
    s_screen = Screen::Detail;
    redraw();
}

static void goUp() {
    if (!s_dir.length()) { feature_exit_requested = true; return; }
    int slash = s_dir.lastIndexOf('/');
    s_dir = (slash >= 0) ? s_dir.substring(0, slash) : String("");
    s_sel = 0; s_first = 0;
    scanDir();
    redraw();
}

static void sendCurrent() {
    if (!s_parsedOk || !s_parsed.ok) { s_status = "Ungueltige Datei"; redraw(); return; }
    if (!SubFlip::isSendable(s_parsed)) {
        s_status = s_parsed.rolling ? "Rolling-Code: gesperrt" : "Nicht sendbar";
        redraw();
        return;
    }
    std::vector<SubGhz::LevelDuration> w = SubFlip::toWaveform(s_parsed);
    if (w.empty()) { s_status = "Nichts zu senden"; redraw(); return; }
    s_status = "Sende...";
    redraw();
    int repeats = s_parsed.raw ? 3 : 6;   // RAW usually already repeats; fixed codes don't
    sfTxWaveform(s_parsed.frequencyHz, w, repeats);
    s_status = "Gesendet";
    redraw();
}

// ---- input -----------------------------------------------------------------
// Hold-to-scroll repeats. The initial move is produced by the button edge in
// the caller; this only adds the auto-repeat after a hold (380ms, then 110ms).
static int scrollStep(bool upLevel, bool downLevel) {
    bool want = upLevel || downLevel;
    int dir = upLevel ? -1 : (downLevel ? +1 : 0);
    uint32_t now = millis();
    if (!want) { s_repActive = false; s_repDir = 0; return 0; }
    if (!s_repActive || dir != s_repDir) {
        s_repActive = true; s_repDir = dir;
        s_repNext = now + 380;             // wait for the hold threshold first
        return 0;
    }
    if (now >= s_repNext) { s_repNext = now + 110; return dir; }
    return 0;
}

static void handleBrowseInput() {
    // edge presses (fires once for both touch-nav and physical)
    bool eUp   = isButtonPressedEdge(BTN_UP);
    bool eDown = isButtonPressedEdge(BTN_DOWN);
    bool eLeft = isButtonPressedEdge(BTN_LEFT);
    bool eCtr  = isButtonPressedEdge(BTN_SELECT);
    bool eRight= isButtonPressedEdge(BTN_RIGHT);

    // auto-repeat on held up/down (level); edge above already consumed first tick,
    // so only use the repeat phase here
    bool lUp   = isButtonPressed(BTN_UP)   || isTouchNavButtonPressed(BTN_UP);
    bool lDown = isButtonPressed(BTN_DOWN) || isTouchNavButtonPressed(BTN_DOWN);
    int step = 0;
    if (eUp)        step = -1;
    else if (eDown) step = +1;
    else            step = scrollStep(lUp, lDown);

    if (step && !s_kids.empty()) {
        s_sel += step;
        if (s_sel < 0) s_sel = 0;
        if (s_sel >= (int)s_kids.size()) s_sel = (int)s_kids.size() - 1;
        redraw();
    }

    if (eLeft) { goUp(); return; }
    if (eCtr || eRight) { openSelected(); return; }

    // tap to select / act
    int tx, ty;
    if (readTouchXY(tx, ty)) {
        if (ty >= kListTop - 2 && ty < listBottomY()) {
            int row = (ty - (kListTop - 2)) / kLineH;
            int idx = s_first + row;
            if (idx >= 0 && idx < (int)s_kids.size()) {
                bool act = (idx == s_sel);
                s_sel = idx;
                if (act) { openSelected(); }
                else redraw();
            }
        }
    }
}

static void handleDetailInput() {
    bool eLeft = isButtonPressedEdge(BTN_LEFT);
    bool eCtr  = isButtonPressedEdge(BTN_SELECT);
    bool eRight= isButtonPressedEdge(BTN_RIGHT);

    if (eLeft) {
        s_screen = Screen::Browse;
        scanDir();
        redraw();
        return;
    }
    if (eCtr || eRight) { sendCurrent(); return; }

    int tx, ty;
    if (readTouchXY(tx, ty)) {
        // tapping the body sends
        if (ty >= kListTop && ty < listBottomY()) sendCurrent();
    }
}

// ---- public ----------------------------------------------------------------
void setup() {
    s_active = true;
    s_screen = Screen::Browse;
    s_dir = "";
    s_sel = 0; s_first = 0;
    s_status = "";
    s_repActive = false; s_repDir = 0;
    feature_exit_requested = false;

    restoreSdAfterSharedSpi();
    scanDir();
    setTouchButtonInputEnabled(true);
    resetTouchNavHeldState();

    // Drain the SELECT press used to enter the feature so the first list row
    // is not opened immediately, then consume any pending button edges.
    uint32_t t0 = millis();
    while ((isButtonPressed(BTN_SELECT) || isTouchNavButtonPressed(BTN_SELECT)) &&
           millis() - t0 < 600) {
        delay(5);
    }
    (void)isButtonPressedEdge(BTN_SELECT);
    (void)isButtonPressedEdge(BTN_UP);
    (void)isButtonPressedEdge(BTN_DOWN);
    (void)isButtonPressedEdge(BTN_LEFT);
    (void)isButtonPressedEdge(BTN_RIGHT);

    redraw();
}

void loop() {
    if (!s_active) return;
    if (feature_exit_requested) return;

    if (s_screen == Screen::Browse) handleBrowseInput();
    else handleDetailInput();

    delay(8);
}

} // namespace SubFlipper
