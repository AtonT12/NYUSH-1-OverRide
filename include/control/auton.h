#pragma once

#include <stdint.h>

#include "control/command.h"

namespace control {

// Bit layout of AutonFrame::flags. Sampling holds a frame's flags across the
// interval that starts at that frame, so the recorder writes them on every tick
// while a hand-written table only needs them where they change.
enum : uint8_t {
  kFlagLiftFinePositive = 1u << 0,
  kFlagLiftFineNegative = 1u << 1,
  kFlagFixturePositive = 1u << 2,
  kFlagFixtureNegative = 1u << 3,
  kFlagToggle = 1u << 4,
  kFlagEndOfProgram = 1u << 5
};

// One wire-format sample. 12 bytes with no padding, so a recorded buffer can be
// memory-mapped, CRC'd and replayed without a serialisation pass.
//
// Continuous channels are stored in permille (int16) rather than degrees or
// floats: 0.1% resolution is far below the motor's own deadband, and it keeps a
// 30 s routine at 20 ms (1500 frames) down to 18 KB.
struct AutonFrame {
  uint16_t tMs;
  int16_t forwardPermille;
  int16_t strafePermille;
  int16_t turnPermille;
  int16_t liftCoarsePermille;
  uint8_t flags;
  uint8_t reserved;
};

static_assert(sizeof(AutonFrame) == 12, "AutonFrame is a 12-byte wire format");

struct AutonProgram {
  const AutonFrame* frames;
  uint16_t count;
  uint16_t periodMs;

  constexpr AutonProgram() : frames(0), count(0), periodMs(20) {}
  constexpr AutonProgram(const AutonFrame* frameTable, uint16_t frameCount,
                         uint16_t loopPeriodMs)
      : frames(frameTable), count(frameCount), periodMs(loopPeriodMs) {}
};

// 30 s of autonomous at 20 ms is 1500 frames; VEX U plays a 30 s autonomous
// period, so this is the largest table a match can consume.
static const uint16_t kAutonMaxFrames = 1500;
static const uint32_t kAutonFrameBytes = 12;
static const uint32_t kAutonHeaderBytes = 16;
static const uint32_t kAutonMaxFileBytes =
    kAutonHeaderBytes + kAutonMaxFrames * kAutonFrameBytes;

// File layout, little-endian:
//   0..3   "OVRR"
//   4      version (1)
//   5      reserved
//   6..7   periodMs
//   8..9   frame count
//   10..11 reserved
//   12..15 CRC-32 of the frame bytes
//   16..   frame bytes

// Writes 12-byte frames back to back into a caller-owned buffer. The recorder
// puts one over its SD buffer; the host tests put one over a local array, which
// is what makes the "which frames are kept" decision testable without a robot.
struct FrameWriter {
  uint8_t* base;      // start of the frame area, i.e. after the 16-byte header
  uint16_t capacity;  // frames
  uint16_t count;

  constexpr FrameWriter() : base(0), capacity(0), count(0) {}
  FrameWriter(uint8_t* buffer, uint16_t frameCapacity)
      : base(buffer), capacity(frameCapacity), count(0) {}
};

// Encodes `command` at `tMs` and appends it. One call is one frame: there is
// deliberately no "same command, extend the previous timestamp" merge, because
// that moves an input change to the end of its hold, and replay then ramps into
// a change that should have been a step. Rejects a full writer and a timestamp
// that goes backwards (the offline encoder refuses those).
bool frameWriterAppend(FrameWriter* writer, const UserCommand& command,
                       uint16_t tMs);

// Rounds a normalized value to permille, clamping outside [-1, 1].
int16_t autonToPermille(double unit);

// Fills one frame from a command. Discrete channels become flag bits.
void autonEncodeFrame(const UserCommand& command, uint16_t tMs, AutonFrame* frame);

// Standard reflected CRC-32 (the same polynomial zlib uses), so the Python
// tools can verify a recording without reimplementing anything.
uint32_t autonCrc32(const uint8_t* data, uint32_t length);

// Writes the 16-byte header for a payload that is already sitting at
// buffer + kAutonHeaderBytes. The recorder encodes frames straight into the
// file buffer, so there is no whole-program serialise step: the firmware never
// reads a .bin back, it only writes one for the offline encoder.
bool autonWriteHeader(uint8_t* buffer, uint32_t capacity, uint32_t payloadBytes,
                      uint16_t count, uint16_t periodMs);

// Last frame's timestamp, or 0 for an empty program.
uint32_t autonDurationMs(const AutonProgram& program);

// Linear interpolation on the four continuous channels; discrete channels hold
// the value of the earlier frame. Out-of-range times clamp to the first/last
// frame, which also makes an empty program a no-op instead of a fault.
UserCommand sampleAuton(const AutonProgram& program, uint32_t tMs);

}  // namespace control
