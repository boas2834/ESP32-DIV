// ir_encoders.cpp — the single translation unit that emits the IrEnc encoder
// implementations. Their bodies live in ir_encoders.h behind
// IR_ENCODERS_IMPL; defining it here (and nowhere else in the firmware) gives
// exactly one definition of each encoder for the linker. Every other file
// includes ir_encoders.h for the declarations only.
#define IR_ENCODERS_IMPL
#include "ir_encoders.h"
