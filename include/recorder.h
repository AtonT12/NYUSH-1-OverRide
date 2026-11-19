#pragma once

#include "control/auton.h"

// Captures the shaped command stream during a Record-mode driver run and saves
// it as a .bin the offline encoder can turn into a program table.
namespace Recorder {

void reset();

// Appends one control tick. Returns false once 30 s or the frame budget is
// reached; the caller keeps driving, it just stops being recorded.
bool push(const control::UserCommand& command, uint32_t tMs);

uint16_t frameCount();
uint32_t byteCount();
bool isFull();

// Writes the header and every frame to the SD card. Returns false if no card is
// inserted or the write was short.
bool save(const char* path);

}  // namespace Recorder
