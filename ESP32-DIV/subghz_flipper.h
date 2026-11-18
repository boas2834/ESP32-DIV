#pragma once
//
// subghz_flipper.h — "Flipper .sub" Sub-GHz feature for ESP32-DIV.
//
// Browses Flipper Zero .sub files on the SD card (under /subghz/, with
// subdirectories), shows the parsed signal, and transmits it on the CC1101 in
// OOK: RAW captures are replayed from their microsecond runs, fixed-code
// protocols (Princeton/CAME/Nice FLO/Holtek) are re-synthesised by the tested
// SubGhz:: encoders. Rolling codes are shown but not transmitted.
//
namespace SubFlipper {
  void setup();
  void loop();
}
