// Single translation unit that emits the SubGhz:: implementations.
// Keeping the impl in exactly one TU avoids multiple-definition linker errors
// (same stb-style pattern as ir_encoders.cpp).
#define SUBGHZ_DECODERS_IMPL
#include "subghz_decoders.h"
