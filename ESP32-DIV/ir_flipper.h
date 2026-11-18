#pragma once
//
// ir_flipper.h — "Flipper .ir" IR feature for ESP32-DIV.
//
// Browses Flipper Zero .ir library files on the SD card (under /ir_flipper/),
// lists the buttons in a file, and transmits a button, or sweeps every stored
// code that shares a button name (a universal remote for that device class).
//
// Transmission always goes out as a raw waveform via IRsend::sendRaw():
// parsed signals are synthesised by IrEnc::encode() (see ir_encoders.h), raw
// signals are replayed from the file. This keeps timing under our control and
// avoids any bit-order mismatch between IR libraries.
//
#include <Arduino.h>
#include <vector>

namespace IRFlipper {
  void setup();
  void loop();

  // --- Reusable, lazy library API (used by the IR Universal Controller) ------
  // All paths are relative to /ir_flipper (no leading slash). Nothing is cached
  // across calls: each call touches the SD card for just what it needs, so the
  // whole multi-thousand-file Flipper-IRDB never has to fit in RAM.

  // True when an /ir_flipper directory exists on the SD card.
  bool libAvailable();

  struct Child { String name; bool isDir; };     // a subfolder or a .ir file
  struct Btn   { String name; uint32_t offset; }; // a button and its byte offset

  // List one directory: subfolders first, then .ir files. False if not a dir.
  bool libList(const String& relDir, std::vector<Child>& out);

  // Index every button (name + SD offset) in one .ir file. False on open error.
  bool libIndex(const String& relFile, std::vector<Btn>& out);

  // Read the record at `offset` in `relFile` and transmit it. Handles both raw
  // and parsed (encodable) Flipper signals. False if read/encode/send failed.
  bool libSend(const String& relFile, uint32_t offset);
}
