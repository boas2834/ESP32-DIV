// ir_flipper.cpp — "Flipper .ir" IR feature. See ir_flipper.h.
//
// Reads Flipper Zero .ir library files from /ir_flipper/ on the SD card and
// transmits their buttons, including a per-button "sweep" across every stored
// code of that name (a universal remote for a device class).
//
// Memory-safe by design: only button names + byte offsets are held in RAM;
// each signal is re-read from SD and encoded on demand just before sending.

#include <Arduino.h>
#include <IRsend.h>
#include <SD.h>
#include <vector>
#include "shared.h"
#include "utils.h"
#include "ir_flipper.h"
#include "ir_flipper_parse.h"
#include "ir_encoders.h"

namespace IRFlipper {

namespace {

constexpr const char* FLIPPER_DIR = "/ir_flipper";
constexpr uint16_t    kMaxFiles   = 200;
constexpr uint16_t    kMaxRecords = 600;
constexpr uint16_t    kGapMs      = 120;   // gap between sweep transmissions

IRsend s_send(IR_TX_PIN);

enum class Screen : uint8_t { Files, Buttons };
Screen s_screen = Screen::Files;

std::vector<String> s_files;      // file names inside /ir_flipper
int s_fileSel = 0;

// One indexed record: its button name and where its "name:" line starts.
struct Entry { String name; uint32_t offset; };
std::vector<Entry>  s_records;    // all records in the open file
std::vector<String> s_sweepNames; // distinct names (sweep targets), in order
String s_openFile;
int s_btnSel = 0;                 // index into the combined display list

bool s_uiDrawn = false;
bool s_dirty   = true;
String s_status;

// Combined display list = [sweep targets...] then [individual records...].
int displayCount() { return (int)s_sweepNames.size() + (int)s_records.size(); }
bool isSweepRow(int row) { return row < (int)s_sweepNames.size(); }
int  recordRow(int row) { return row - (int)s_sweepNames.size(); }

// ---- SD helpers ----

bool endsWithIr(const String& n) {
  int l = n.length();
  return l >= 3 && (n[l-3]=='.') &&
         (n[l-2]=='i'||n[l-2]=='I') && (n[l-1]=='r'||n[l-1]=='R');
}

void scanFiles() {
  s_files.clear();
  if (!SD.exists(FLIPPER_DIR)) { SD.mkdir(FLIPPER_DIR); }
  File dir = SD.open(FLIPPER_DIR);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
  File f;
  while ((f = dir.openNextFile()) && s_files.size() < kMaxFiles) {
    if (!f.isDirectory()) {
      String n = f.name();
      int slash = n.lastIndexOf('/');
      if (slash >= 0) n = n.substring(slash + 1);
      if (endsWithIr(n)) s_files.push_back(n);
    }
    f.close();
  }
  dir.close();
}

// Index every record in a file: store each button name and its byte offset.
void indexFile(const String& fname) {
  s_records.clear();
  s_sweepNames.clear();
  s_openFile = fname;
  String path = String(FLIPPER_DIR) + "/" + fname;
  File f = SD.open(path, FILE_READ);
  if (!f) { s_status = "Open failed"; return; }
  uint32_t pos = f.position();
  while (f.available() && s_records.size() < kMaxRecords) {
    String line = f.readStringUntil('\n');
    // A record is anchored at its "name:" line.
    if (line.startsWith("name:")) {
      String nm = line.substring(5);
      nm.trim();
      s_records.push_back({nm, pos});
      // Track distinct names for the sweep list (preserve first-seen order).
      bool seen = false;
      for (auto& s : s_sweepNames) if (s == nm) { seen = true; break; }
      if (!seen) s_sweepNames.push_back(nm);
    }
    pos = f.position();
  }
  f.close();
  // Only offer sweep rows when a name actually repeats across the file; a file
  // with all-unique names gains nothing from them.
  if (s_sweepNames.size() == s_records.size()) s_sweepNames.clear();
}

// Read exactly one record starting at a byte offset.
bool readRecordAt(uint32_t offset, IrFlip::Record& rec) {
  String path = String(FLIPPER_DIR) + "/" + s_openFile;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  f.seek(offset);
  IrFlip::Reader r([&f](std::string& out) -> bool {
    if (!f.available()) return false;
    String s = f.readStringUntil('\n');
    out = s.c_str();  // Arduino String -> std::string
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
    std::vector<uint16_t> t = rec.rawTimings;  // sendRaw wants a mutable buffer
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

// Sweep: send every record whose name matches, with a short gap between them.
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

void drawHeader(const char* title) {
  tft.fillRect(0, 20, 240, 20, FEATURE_BG);
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(6, 24);
  tft.print(title);
  tft.drawFastHLine(0, 40, 240, UI_LINE);
}

void drawStatusLine() {
  tft.fillRect(0, 300, 240, 16, FEATURE_BG);
  tft.setTextColor(UI_WARN, FEATURE_BG);
  tft.setCursor(6, 302);
  String s = s_status;
  if (s.length() > 36) s = s.substring(0, 36);
  tft.print(s);
}

template <typename GetLabel>
void drawList(int count, int sel, GetLabel label) {
  int first = sel - kRows / 2;
  if (first < 0) first = 0;
  if (first > count - kRows) first = count - kRows;
  if (first < 0) first = 0;
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

void redrawFiles() {
  drawHeader("Flipper .ir  -  select file");
  if (s_files.empty()) {
    tft.fillRect(0, kListTop, 240, kRows * kLineH, FEATURE_BG);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(8, kListTop + 4);
    tft.print("No .ir files in /ir_flipper");
  } else {
    drawList((int)s_files.size(), s_fileSel, [](int i){ return s_files[i]; });
  }
  s_status = "UP/DN move  RIGHT open";
  drawStatusLine();
}

String displayLabel(int row) {
  if (isSweepRow(row)) return String("[Sweep] ") + s_sweepNames[row];
  return s_records[recordRow(row)].name;
}

void redrawButtons() {
  String h = "File: " + s_openFile;
  drawHeader(h.c_str());
  drawList(displayCount(), s_btnSel, displayLabel);
  s_status = "RIGHT send  LEFT back";
  drawStatusLine();
}

} // anonymous namespace

void setup() {
  setTouchButtonInputEnabled(true);
  s_send.begin();
  tft.fillScreen(FEATURE_BG);
  float v = readBatteryVoltage();
  drawStatusBar(v, true);
  s_screen = Screen::Files;
  s_fileSel = 0;
  s_btnSel = 0;
  scanFiles();
  s_uiDrawn = false;
  s_dirty = true;
}

void loop() {
  if (feature_active && (feature_exit_requested || featureExitButtonPressed())) {
    feature_exit_requested = true;
    return;
  }
  redrawTouchButtonBar();

  if (s_dirty || !s_uiDrawn) {
    if (s_screen == Screen::Files) redrawFiles();
    else                          redrawButtons();
    s_uiDrawn = true;
    s_dirty = false;
  }

  static uint32_t lastMs = 0;
  const uint32_t debounce = 180;
  static bool pUp=false, pDn=false, pR=false, pL=false;
  bool up = isPhysicalButtonPressed(BTN_UP);
  bool dn = isPhysicalButtonPressed(BTN_DOWN);
  bool r  = isPhysicalButtonPressed(BTN_RIGHT);
  bool l  = isPhysicalButtonPressed(BTN_LEFT);
  uint32_t now = millis();

  if (s_screen == Screen::Files) {
    int n = (int)s_files.size();
    if (dn && !pDn && now - lastMs > debounce && n) { s_fileSel = (s_fileSel+1)%n; s_dirty=true; lastMs=now; }
    if (up && !pUp && now - lastMs > debounce && n) { s_fileSel = (s_fileSel-1+n)%n; s_dirty=true; lastMs=now; }
    if (r && !pR && now - lastMs > debounce && n) {
      indexFile(s_files[s_fileSel]);
      s_btnSel = 0;
      s_screen = Screen::Buttons;
      s_dirty = true; lastMs = now;
    }
  } else {
    int n = displayCount();
    if (dn && !pDn && now - lastMs > debounce && n) { s_btnSel = (s_btnSel+1)%n; s_dirty=true; lastMs=now; }
    if (up && !pUp && now - lastMs > debounce && n) { s_btnSel = (s_btnSel-1+n)%n; s_dirty=true; lastMs=now; }
    if (l && !pL && now - lastMs > debounce) {
      s_screen = Screen::Files; s_dirty = true; lastMs = now;
    }
    if (r && !pR && now - lastMs > debounce && n) {
      if (isSweepRow(s_btnSel)) sweepName(s_sweepNames[s_btnSel]);
      else sendOneOffset(s_records[recordRow(s_btnSel)].offset);
      drawStatusLine();
      lastMs = now;
    }
  }

  pUp=up; pDn=dn; pR=r; pL=l;
}

} // namespace IRFlipper
