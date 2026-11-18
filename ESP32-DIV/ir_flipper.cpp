// ir_flipper.cpp — "Flipper .ir" IR feature. See ir_flipper.h.
//
// Browses Flipper Zero .ir library files on the SD card under /ir_flipper/,
// descending into subdirectories (so a full tree like the Flipper-IRDB can be
// dropped in as-is), lists the buttons in a file, and transmits a button or
// sweeps every stored code that shares a button name (a universal remote for
// a device class).
//
// Transmission always goes out as a raw waveform via IRsend::sendRaw():
// parsed signals are synthesised by IrEnc::encode() (ir_encoders.h), raw
// signals are replayed from the file. Memory-safe: only directory entries /
// button names + SD byte offsets are held in RAM; each signal is re-read from
// SD and encoded on demand just before sending.

#include <Arduino.h>
#include <IRsend.h>
#include <SD.h>
#include <vector>
#include "shared.h"
#include "utils.h"
#include "Touchscreen.h"
#include "ir_flipper.h"
#include "ir_flipper_parse.h"
#include "ir_encoders.h"

namespace IRFlipper {

namespace {

constexpr const char* FLIPPER_DIR = "/ir_flipper";
constexpr uint16_t    kMaxEntries = 600;
constexpr uint16_t    kMaxRecords = 600;
constexpr uint16_t    kGapMs      = 120;   // gap between sweep transmissions

IRsend s_send(IR_TX_PIN);

enum class Screen : uint8_t { Browse, Buttons };
Screen s_screen = Screen::Browse;

// ---- directory browser state ----
struct DirEntry { String name; bool isDir; };
std::vector<DirEntry> s_entries;
String s_curDir = "";      // relative to FLIPPER_DIR, no leading/trailing '/'
int    s_browseSel = 0;

// ---- open-file state ----
struct Rec { String name; uint32_t offset; };
std::vector<Rec>    s_records;     // all records in the open file
std::vector<String> s_sweepNames;  // distinct names (sweep targets), in order
String s_openFile;                 // relative to FLIPPER_DIR
int    s_btnSel = 0;

bool   s_uiDrawn = false;
bool   s_dirty   = true;
String s_status;

String fullDir() {
  return s_curDir.length() ? String(FLIPPER_DIR) + "/" + s_curDir : String(FLIPPER_DIR);
}
String relJoin(const String& base, const String& leaf) {
  return base.length() ? base + "/" + leaf : leaf;
}
// Drop the last '/'-separated segment of a relative path.
String relParent(const String& p) {
  int s = p.lastIndexOf('/');
  return s < 0 ? String("") : p.substring(0, s);
}

bool endsWithIr(const String& n) {
  int l = n.length();
  return l >= 3 && (n[l-3]=='.') &&
         (n[l-2]=='i'||n[l-2]=='I') && (n[l-1]=='r'||n[l-1]=='R');
}

// Combined list = [sweep targets...] then [individual records...].
int displayCount() { return (int)s_sweepNames.size() + (int)s_records.size(); }
bool isSweepRow(int row) { return row < (int)s_sweepNames.size(); }
int  recordRow(int row) { return row - (int)s_sweepNames.size(); }

// ---- SD helpers ----

// List the current directory: ".." (unless root), then subdirs, then .ir files.
void scanDir() {
  s_entries.clear();
  if (s_curDir.length() == 0 && !SD.exists(FLIPPER_DIR)) SD.mkdir(FLIPPER_DIR);
  if (s_curDir.length()) s_entries.push_back({"..", true});
  std::vector<DirEntry> files;
  File dir = SD.open(fullDir());
  if (dir && dir.isDirectory()) {
    File f;
    while ((f = dir.openNextFile()) && (s_entries.size() + files.size()) < kMaxEntries) {
      String n = f.name();
      int slash = n.lastIndexOf('/');
      if (slash >= 0) n = n.substring(slash + 1);
      bool isDir = f.isDirectory();
      f.close();
      if (n.length() == 0) continue;
      if (isDir) s_entries.push_back({n, true});     // dirs first (after "..")
      else if (endsWithIr(n)) files.push_back({n, false});
    }
  }
  if (dir) dir.close();
  for (auto& e : files) s_entries.push_back(e);       // then files
}

// Index every record in a file: store each button name and its byte offset.
void indexFile(const String& relPath) {
  s_records.clear();
  s_sweepNames.clear();
  s_openFile = relPath;
  String path = String(FLIPPER_DIR) + "/" + relPath;
  File f = SD.open(path, FILE_READ);
  if (!f) { s_status = "Open failed"; return; }
  uint32_t pos = f.position();
  while (f.available() && s_records.size() < kMaxRecords) {
    String line = f.readStringUntil('\n');
    if (line.startsWith("name:")) {
      String nm = line.substring(5);
      nm.trim();
      s_records.push_back({nm, pos});
      bool seen = false;
      for (auto& s : s_sweepNames) if (s == nm) { seen = true; break; }
      if (!seen) s_sweepNames.push_back(nm);
    }
    pos = f.position();
  }
  f.close();
  // Sweep rows only help when a name actually repeats in the file.
  if (s_sweepNames.size() == s_records.size()) s_sweepNames.clear();
}

bool readRecordAt(uint32_t offset, IrFlip::Record& rec) {
  String path = String(FLIPPER_DIR) + "/" + s_openFile;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  f.seek(offset);
  IrFlip::Reader r([&f](std::string& out) -> bool {
    if (!f.available()) return false;
    String s = f.readStringUntil('\n');
    out = s.c_str();
    return true;
  });
  bool ok = r.next(rec);
  f.close();
  return ok;
}

// ---- transmission ----

bool sendRecord(const IrFlip::Record& rec) {
  if (rec.raw) {
    if (rec.rawTimings.size() < 2) return false;
    std::vector<uint16_t> t = rec.rawTimings;
    s_send.sendRaw(t.data(), t.size(), rec.frequencyHz / 1000);
    return true;
  }
  if (!IrEnc::isEncodable(rec.proto)) return false;
  IrEnc::Signal sig = IrEnc::encode(rec.proto, rec.addr, rec.cmd);
  if (!sig.ok || sig.timings.size() < 2) return false;
  s_send.sendRaw(sig.timings.data(), sig.timings.size(), sig.frequencyHz / 1000);
  return true;
}

void sendOneOffset(uint32_t offset) {
  IrFlip::Record rec;
  if (!readRecordAt(offset, rec)) { s_status = "Read error"; return; }
  if (sendRecord(rec)) s_status = "Sent: " + String(rec.name.c_str());
  else s_status = "Skipped (" + String(rec.protoName.c_str()) + ")";
}

void sweepName(const String& name) {
  int sent = 0;
  for (auto& e : s_records) {
    if (feature_exit_requested || featureExitButtonPressed()) break;
    if (e.name == name) {
      IrFlip::Record rec;
      if (readRecordAt(e.offset, rec) && sendRecord(rec)) {
        sent++;
        s_status = "Sweep " + name + " (" + String(sent) + ")";
        delay(kGapMs);
      }
    }
  }
  s_status = "Swept " + name + ": " + String(sent);
}

// ---- drawing ----

constexpr int kListTop = 44;
constexpr int kLineH   = 16;
constexpr int kRows    = 9;

// Y of the status line: just above the touch nav bar when it's shown, so the
// two never overlap; otherwise near the bottom of the screen.
int statusY() {
  int bottom = featureHasTouchNavBar() ? (int)touchNavContentBottomY() : 316;
  return bottom - kLineH;
}

// First visible row index for a list of `count` items with cursor `sel`.
// Kept identical to drawList() so touch hit-testing maps to the same rows.
int listFirst(int count, int sel) {
  int first = sel - kRows / 2;
  if (first > count - kRows) first = count - kRows;
  if (first < 0) first = 0;
  return first;
}

// If a tap falls on a visible list row, return its index, else -1.
int rowAtTouchY(int count, int sel, int ty) {
  if (ty < kListTop || ty >= kListTop + kRows * kLineH) return -1;
  int i = (ty - kListTop) / kLineH;
  int idx = listFirst(count, sel) + i;
  return (idx >= 0 && idx < count) ? idx : -1;
}

void drawHeader(const String& title) {
  tft.fillRect(0, 20, 240, 20, FEATURE_BG);
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(6, 24);
  String t = title;
  if (t.length() > 38) t = t.substring(0, 38);
  tft.print(t);
  tft.drawFastHLine(0, 40, 240, UI_LINE);
}

void drawStatusLine() {
  int y = statusY();
  tft.fillRect(0, y, 240, kLineH, FEATURE_BG);
  tft.setTextColor(UI_WARN, FEATURE_BG);
  tft.setCursor(6, y + 2);
  String s = s_status;
  if (s.length() > 36) s = s.substring(0, 36);
  tft.print(s);
}

template <typename GetLabel>
void drawList(int count, int sel, GetLabel label) {
  int first = listFirst(count, sel);
  tft.fillRect(0, kListTop, 240, kRows * kLineH, FEATURE_BG);
  for (int i = 0; i < kRows; ++i) {
    int idx = first + i;
    if (idx >= count) break;
    int y = kListTop + i * kLineH;
    bool cur = (idx == sel);
    if (cur) tft.fillRect(0, y, 240, kLineH, UI_LINE);
    tft.setTextColor(cur ? FEATURE_BG : UI_TEXT, cur ? UI_LINE : FEATURE_BG);
    tft.setCursor(8, y + 3);
    String s = label(idx);
    if (s.length() > 36) s = s.substring(0, 36);
    tft.print(s);
  }
}

void redrawBrowse() {
  drawHeader(String("Flipper .ir  /") + s_curDir);
  if (s_entries.empty()) {
    tft.fillRect(0, kListTop, 240, kRows * kLineH, FEATURE_BG);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(8, kListTop + 4);
    tft.print("Empty. Put .ir files in /ir_flipper");
  } else {
    drawList((int)s_entries.size(), s_browseSel, [](int i) {
      const DirEntry& e = s_entries[i];
      return e.isDir ? (String("[") + e.name + "]") : e.name;
    });
  }
  if (featureHasTouchNavBar()) {
    // slots: left, down, center, up, right. Center (SELECT) = Open now; LEFT is
    // "one level up", or "Exit" at the top folder.
    setTouchNavLabels(s_curDir.length() ? "Hoch" : "Exit",
                      "Runter", "Oeffnen", "Hoch", "Oeffnen");
    redrawTouchButtonBar();
  }
  s_status = "Oeffnen=Ordner/Datei  Hoch=zurueck";
  drawStatusLine();
}

String displayLabel(int row) {
  if (isSweepRow(row)) return String("[Sweep] ") + s_sweepNames[row];
  return s_records[recordRow(row)].name;
}

void redrawButtons() {
  int slash = s_openFile.lastIndexOf('/');
  String fn = slash >= 0 ? s_openFile.substring(slash + 1) : s_openFile;
  drawHeader(String("File: ") + fn);
  drawList(displayCount(), s_btnSel, displayLabel);
  if (featureHasTouchNavBar()) {
    // Center (SELECT) = Send now; LEFT goes back to the folder list.
    setTouchNavLabels("Zurueck", "Runter", "Senden", "Hoch", "Senden");
    redrawTouchButtonBar();
  }
  s_status = "Senden  Zurueck=Liste";
  drawStatusLine();
}

// Held UP/DOWN auto-repeat for list scrolling (see the same helper in ir.cpp):
// returns -1/+1 on first press, then repeatedly while held (after a short delay,
// faster on a long hold). 0 when no step should happen this pass.
int listStep() {
  static int dir = 0;
  static uint32_t downAt = 0, lastRep = 0;
  const bool up = isButtonPressed(BTN_UP)   || isTouchNavButtonPressed(BTN_UP);
  const bool dn = isButtonPressed(BTN_DOWN) || isTouchNavButtonPressed(BTN_DOWN);
  const int d = up ? -1 : (dn ? 1 : 0);
  const uint32_t now = millis();
  if (d == 0) { dir = 0; return 0; }
  if (d != dir) { dir = d; downAt = now; lastRep = now; return d; }
  if (now - downAt < 400) return 0;
  const uint32_t interval = (now - downAt > 2000) ? 45 : 110;
  if (now - lastRep >= interval) { lastRep = now; return d; }
  return 0;
}

} // anonymous namespace

// ---- Reusable library API (see ir_flipper.h) -----------------------------
// These are stateless with respect to the interactive browser above: they take
// explicit paths and never touch s_curDir/s_records, so the Universal
// Controller can call them without disturbing (or being disturbed by) a browse
// session. They reuse the anonymous-namespace helpers (FLIPPER_DIR, endsWithIr,
// sendRecord, …) from the same translation unit.

bool libAvailable() {
  return SD.exists(FLIPPER_DIR);
}

bool libList(const String& relDir, std::vector<Child>& out) {
  out.clear();
  String full = relDir.length() ? String(FLIPPER_DIR) + "/" + relDir : String(FLIPPER_DIR);
  File dir = SD.open(full);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return false; }
  std::vector<Child> files;
  File f;
  while ((f = dir.openNextFile()) && (out.size() + files.size()) < kMaxEntries) {
    String n = f.name();
    int slash = n.lastIndexOf('/');
    if (slash >= 0) n = n.substring(slash + 1);
    bool isDir = f.isDirectory();
    f.close();
    if (n.length() == 0) continue;
    if (isDir) out.push_back({n, true});
    else if (endsWithIr(n)) files.push_back({n, false});
  }
  dir.close();
  for (auto& e : files) out.push_back(e);
  return true;
}

bool libIndex(const String& relFile, std::vector<Btn>& out) {
  out.clear();
  String path = String(FLIPPER_DIR) + "/" + relFile;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  uint32_t pos = f.position();
  while (f.available() && out.size() < kMaxRecords) {
    String line = f.readStringUntil('\n');
    if (line.startsWith("name:")) {
      String nm = line.substring(5);
      nm.trim();
      out.push_back({nm, pos});
    }
    pos = f.position();
  }
  f.close();
  return true;
}

bool libSend(const String& relFile, uint32_t offset) {
  String path = String(FLIPPER_DIR) + "/" + relFile;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  f.seek(offset);
  IrFlip::Record rec;
  IrFlip::Reader r([&f](std::string& o) -> bool {
    if (!f.available()) return false;
    o = f.readStringUntil('\n').c_str();
    return true;
  });
  bool ok = r.next(rec);
  f.close();
  if (!ok) return false;
  return sendRecord(rec);
}

void setup() {
  setTouchButtonInputEnabled(true);
  s_send.begin();
  tft.fillScreen(FEATURE_BG);
  drawStatusBar(readBatteryVoltage(), true);
  s_screen = Screen::Browse;
  s_curDir = "";
  s_browseSel = 0;
  s_btnSel = 0;
  scanDir();
  s_uiDrawn = false;
  s_dirty = true;
}

void loop() {
  // The center/SELECT button is repurposed as Open/Send here (see below), so
  // exit only on an explicit request: Back at the top folder, or the center
  // slot label "Exit" is no longer used. Leaving happens via BTN_LEFT at root.
  if (feature_active && feature_exit_requested) {
    return;
  }
  redrawTouchButtonBar();

  if (s_dirty || !s_uiDrawn) {
    if (s_screen == Screen::Browse) redrawBrowse();
    else                            redrawButtons();
    s_uiDrawn = true;
    s_dirty = false;
  }

  // Edge-triggered input: isButtonPressedEdge() fires once per press for BOTH
  // the on-screen touch nav bar and the physical PCF buttons, so every action
  // works by touch or by button. The center/SELECT button now Opens/Sends (it
  // is no longer a blanket feature-exit); LEFT goes up a level and exits at the
  // top folder.
  int  step = listStep();                         // held UP/DOWN auto-repeats
  bool up  = (step < 0);
  bool dn  = (step > 0);
  bool rRight = isButtonPressedEdge(BTN_RIGHT);
  bool rSel   = isButtonPressedEdge(BTN_SELECT);  // evaluate both to consume edges
  bool r   = rRight || rSel;
  bool l   = isButtonPressedEdge(BTN_LEFT);

  // Direct tap on a list row: select it, and tapping the already-selected row
  // acts on it (open folder/file in Browse, send in Buttons).
  static uint32_t lastTapMs = 0;
  int tapRow = -1;
  {
    int tx, ty;
    uint32_t now = millis();
    if (now - lastTapMs > 220 && readTouchXY(tx, ty)) {
      int cnt = (s_screen == Screen::Browse) ? (int)s_entries.size() : displayCount();
      int selNow = (s_screen == Screen::Browse) ? s_browseSel : s_btnSel;
      tapRow = rowAtTouchY(cnt, selNow, ty);
      if (tapRow >= 0) lastTapMs = now;
    }
  }

  if (s_screen == Screen::Browse) {
    int n = (int)s_entries.size();
    if (dn && n) { s_browseSel = (s_browseSel+1)%n; s_dirty=true; }
    if (up && n) { s_browseSel = (s_browseSel-1+n)%n; s_dirty=true; }
    if (l) {
      if (s_curDir.length()) { s_curDir = relParent(s_curDir); s_browseSel = 0; scanDir(); s_dirty = true; }
      else { feature_exit_requested = true; return; }   // Back at the top folder leaves
    }
    bool tapAct = (tapRow >= 0 && tapRow == s_browseSel);   // tap on selected row
    if (tapRow >= 0 && tapRow != s_browseSel) { s_browseSel = tapRow; s_dirty = true; }
    bool open = (r && n) || tapAct;
    if (open) {
      const DirEntry& e = s_entries[s_browseSel];
      if (e.isDir) {
        if (e.name == "..") s_curDir = relParent(s_curDir);
        else                s_curDir = relJoin(s_curDir, e.name);
        s_browseSel = 0; scanDir(); s_dirty = true;
      } else {
        indexFile(relJoin(s_curDir, e.name));
        s_btnSel = 0; s_screen = Screen::Buttons; s_dirty = true;
      }
    }
  } else {
    int n = displayCount();
    if (dn && n) { s_btnSel = (s_btnSel+1)%n; s_dirty=true; }
    if (up && n) { s_btnSel = (s_btnSel-1+n)%n; s_dirty=true; }
    if (l) { s_screen = Screen::Browse; s_dirty = true; }
    bool tapAct = (tapRow >= 0 && tapRow == s_btnSel && n);  // tap on selected row
    if (tapRow >= 0 && tapRow != s_btnSel) { s_btnSel = tapRow; s_dirty = true; }
    bool send = (r && n) || tapAct;
    if (send) {
      if (isSweepRow(s_btnSel)) sweepName(s_sweepNames[s_btnSel]);
      else sendOneOffset(s_records[recordRow(s_btnSel)].offset);
      drawStatusLine();
    }
  }
}

} // namespace IRFlipper
