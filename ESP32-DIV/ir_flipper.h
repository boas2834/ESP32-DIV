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
namespace IRFlipper {
  void setup();
  void loop();
}
