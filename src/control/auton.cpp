#include "control/auton.h"

#include <cstring>

namespace control {
namespace {

double permilleToUnit(int16_t value) {
  return static_cast<double>(value) / 1000.0;
}

void applyFlags(UserCommand* command, uint8_t flags) {
  command->liftFine = 0;
  if ((flags & kFlagLiftFinePositive) != 0) {
    command->liftFine = 1;
  } else if ((flags & kFlagLiftFineNegative) != 0) {
    command->liftFine = -1;
  }

  command->fixture = 0;
  if ((flags & kFlagFixturePositive) != 0) {
    command->fixture = 1;
  } else if ((flags & kFlagFixtureNegative) != 0) {
    command->fixture = -1;
  }

  command->toggle = (flags & kFlagToggle) != 0;
}

void applyFrame(UserCommand* command, const AutonFrame& frame) {
  command->forward = permilleToUnit(frame.forwardPermille);
  command->strafe = permilleToUnit(frame.strafePermille);
  command->turn = permilleToUnit(frame.turnPermille);
  command->liftCoarse = permilleToUnit(frame.liftCoarsePermille);
  applyFlags(command, frame.flags);
}

double lerp(double a, double b, double u) {
  return a + (b - a) * u;
}

void writeU16(uint8_t* p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFFu);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void writeU32(uint8_t* p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFFu);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  p[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

const uint8_t kMagic[4] = {'O', 'V', 'R', 'R'};
const uint8_t kVersion = 1;

}  // namespace

uint32_t autonDurationMs(const AutonProgram& program) {
  if (program.frames == 0 || program.count == 0) {
    return 0;
  }
  return program.frames[program.count - 1].tMs;
}

UserCommand sampleAuton(const AutonProgram& program, uint32_t tMs) {
  UserCommand command;
  if (program.frames == 0 || program.count == 0) {
    return command;
  }

  const AutonFrame& first = program.frames[0];
  if (tMs <= first.tMs) {
    applyFrame(&command, first);
    return command;
  }

  const AutonFrame& last = program.frames[program.count - 1];
  if (tMs >= last.tMs) {
    applyFrame(&command, last);
    return command;
  }

  // Linear scan rather than binary search: 1500 frames at 50 Hz is a handful
  // of microseconds on a 667 MHz core, and the scan keeps the replay allocation
  // free and branch-predictable.
  uint16_t i = 0;
  while (static_cast<uint32_t>(i) + 1 < program.count &&
         program.frames[i + 1].tMs <= tMs) {
    ++i;
  }

  const AutonFrame& a = program.frames[i];
  const AutonFrame& b = program.frames[i + 1];
  const uint32_t span = static_cast<uint32_t>(b.tMs) - a.tMs;
  const double u = span == 0
                       ? 1.0
                       : static_cast<double>(tMs - a.tMs) / static_cast<double>(span);

  command.forward = lerp(permilleToUnit(a.forwardPermille),
                         permilleToUnit(b.forwardPermille), u);
  command.strafe =
      lerp(permilleToUnit(a.strafePermille), permilleToUnit(b.strafePermille), u);
  command.turn = lerp(permilleToUnit(a.turnPermille), permilleToUnit(b.turnPermille), u);
  command.liftCoarse = lerp(permilleToUnit(a.liftCoarsePermille),
                            permilleToUnit(b.liftCoarsePermille), u);

  // Discrete channels hold the earlier frame's value across the whole interval,
  // so a flag only has to be written on the frame where it changes.
  applyFlags(&command, a.flags);
  return command;
}

int16_t autonToPermille(double unit) {
  double clamped = unit;
  if (clamped > 1.0) {
    clamped = 1.0;
  }
  if (clamped < -1.0) {
    clamped = -1.0;
  }
  const double scaled = clamped * 1000.0;
  const long rounded = static_cast<long>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
  return static_cast<int16_t>(rounded);
}

void autonEncodeFrame(const UserCommand& command, uint16_t tMs, AutonFrame* frame) {
  if (frame == 0) {
    return;
  }
  frame->tMs = tMs;
  frame->forwardPermille = autonToPermille(command.forward);
  frame->strafePermille = autonToPermille(command.strafe);
  frame->turnPermille = autonToPermille(command.turn);
  frame->liftCoarsePermille = autonToPermille(command.liftCoarse);

  uint8_t flags = 0;
  if (command.liftFine > 0) {
    flags |= kFlagLiftFinePositive;
  } else if (command.liftFine < 0) {
    flags |= kFlagLiftFineNegative;
  }
  if (command.fixture > 0) {
    flags |= kFlagFixturePositive;
  } else if (command.fixture < 0) {
    flags |= kFlagFixtureNegative;
  }
  if (command.toggle) {
    flags |= kFlagToggle;
  }
  frame->flags = flags;
  frame->reserved = 0;
}

bool frameWriterAppend(FrameWriter* writer, const UserCommand& command,
                       uint16_t tMs) {
  if (writer == 0 || writer->base == 0) {
    return false;
  }
  if (writer->count >= writer->capacity) {
    return false;
  }
  if (writer->count > 0) {
    // The wire format stores the timestamp little-endian; read it back with the
    // same explicit byte order so this does not depend on struct layout.
    const uint8_t* last = writer->base + (writer->count - 1) * kAutonFrameBytes;
    const uint16_t previous =
        static_cast<uint16_t>(static_cast<uint16_t>(last[0]) |
                              (static_cast<uint16_t>(last[1]) << 8));
    if (tMs < previous) {
      return false;
    }
  }

  AutonFrame frame;
  autonEncodeFrame(command, tMs, &frame);
  std::memcpy(writer->base + writer->count * kAutonFrameBytes, &frame,
              kAutonFrameBytes);
  ++writer->count;
  return true;
}

uint32_t autonCrc32(const uint8_t* data, uint32_t length) {
  if (data == 0) {
    return 0;
  }
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

bool autonWriteHeader(uint8_t* buffer, uint32_t capacity, uint32_t payloadBytes,
                      uint16_t count, uint16_t periodMs) {
  if (buffer == 0 || payloadBytes > kAutonMaxFrames * kAutonFrameBytes) {
    return false;
  }
  if (kAutonHeaderBytes + payloadBytes > capacity) {
    return false;
  }

  buffer[0] = kMagic[0];
  buffer[1] = kMagic[1];
  buffer[2] = kMagic[2];
  buffer[3] = kMagic[3];
  buffer[4] = kVersion;
  buffer[5] = 0;
  writeU16(buffer + 6, periodMs);
  writeU16(buffer + 8, count);
  writeU16(buffer + 10, 0);
  writeU32(buffer + 12, autonCrc32(buffer + kAutonHeaderBytes, payloadBytes));
  return true;
}

}  // namespace control
