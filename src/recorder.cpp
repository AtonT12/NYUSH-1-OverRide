#include "recorder.h"

#include "constants.h"
#include "robot-config.h"

namespace {

// Frames are encoded in place at the tail of the file buffer, so a save is just
// a header write plus one SD call and never needs a second 18 KB buffer.
alignas(4) uint8_t gBuffer[control::kAutonMaxFileBytes];
control::FrameWriter gWriter(gBuffer + control::kAutonHeaderBytes,
                             control::kAutonMaxFrames);

const uint32_t kMaxRecordMs = 30000u;  // VEX U autonomous period

// The frame budget only covers a match if the control period is 20 ms or
// longer. A faster loop needs a bigger table, not a silently truncated run.
static_assert(RobotConstants::kControlLoopPeriodMs * control::kAutonMaxFrames >=
                  30000,
              "kAutonMaxFrames no longer covers the 30 s autonomous period");

}  // namespace

namespace Recorder {

void reset() {
  gWriter.count = 0;
}

bool push(const control::UserCommand& command, uint32_t tMs) {
  if (tMs > kMaxRecordMs) {
    return false;
  }
  return control::frameWriterAppend(&gWriter, command,
                                    static_cast<uint16_t>(tMs));
}

uint16_t frameCount() {
  return gWriter.count;
}

uint32_t byteCount() {
  return static_cast<uint32_t>(gWriter.count) * control::kAutonFrameBytes;
}

bool save(const char* path) {
  if (path == 0 || gWriter.count == 0) {
    return false;
  }
  const uint32_t payload = byteCount();
  if (!control::autonWriteHeader(
          gBuffer, sizeof(gBuffer), payload, gWriter.count,
          static_cast<uint16_t>(RobotConstants::kControlLoopPeriodMs))) {
    return false;
  }
  if (!Brain.SDcard.isInserted()) {
    return false;
  }
  const int32_t total =
      static_cast<int32_t>(control::kAutonHeaderBytes + payload);
  return Brain.SDcard.savefile(path, gBuffer, total) == total;
}

}  // namespace Recorder
