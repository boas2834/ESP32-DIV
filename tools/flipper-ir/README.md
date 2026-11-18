# Flipper `.ir` support for ESP32-DIV

Lets the ESP32-DIV read Flipper Zero `.ir` library files from the SD card and
transmit them, plus a "Universal Sweep" that tries every stored code for one
button across a device class.

## Layout
- `ESP32-DIV/ir_encoders.h` — pure, hardware-independent protocol encoders
  (Flipper `protocol`/`address`/`command` -> raw mark/space timings). Fed into
  `IRsend::sendRaw()` on the device, which avoids any cross-library bit-order
  mismatch.
- `ESP32-DIV/ir_flipper_parse.h` — streaming parser for the `.ir` text format
  (one record in memory at a time, SD-friendly).
- `tools/flipper-ir/test_ir.cpp` — host unit tests for the two headers above.

## Supported parsed protocols
NEC, NECext, NEC42, Samsung32, SIRC, SIRC15, SIRC20, RC5, RC5X.
Any `type: raw` record is always sendable. Other parsed protocols (Kaseikyo,
RC6, RCA, Pioneer) are recognised and shown, but only transmitted when the file
also carries a raw capture; native encoders for them are a follow-up.

## Running the tests (host, no board needed)
```
cd tools/flipper-ir
g++ -std=c++17 -I../../ESP32-DIV test_ir.cpp -o test_ir && ./test_ir
```
