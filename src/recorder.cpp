#include "recorder.h"

#include "robot-config.h"

#include <cstring>

namespace {

// Frames are encoded in place at the tail of the file buffer, so a save is just
// a header write plus one SD call and never needs a second 18 KB buffer.
alignas(4) uint8_t gBuffer[control::kAutonMaxFileBytes];
uint16_t gCount = 0;

const uint16_t kPeriodMs = 20;
const uint32_t kMaxRecordMs = 30000u;  // VEX U autonomous period

uint8_t* framePtr(uint16_t index) {
  return gBuffer + control::kAutonHeaderBytes +
         static_cast<uint32_t>(index) * control::kAutonFrameBytes;
}

}  // namespace

namespace Recorder {

void reset() {
  gCount = 0;
}

bool push(const control::UserCommand& command, uint32_t tMs) {
  if (tMs > kMaxRecordMs) {
    return false;
  }

  control::AutonFrame frame;
  control::autonEncodeFrame(command, static_cast<uint16_t>(tMs), &frame);

  if (gCount > 0) {
    uint8_t* last = framePtr(static_cast<uint16_t>(gCount - 1));
    // Compare everything except the timestamp. An unchanged command extends the
    // previous frame instead of taking a slot, so holding a stick for two
    // seconds costs one frame rather than a hundred.
    if (std::memcmp(last + 2, reinterpret_cast<const uint8_t*>(&frame) + 2,
                    control::kAutonFrameBytes - 2) == 0) {
      // Little-endian on both the ARM target and the x86 host, matching the wire
      // format, but written explicitly so this does not depend on struct layout.
      last[0] = static_cast<uint8_t>(frame.tMs & 0xFFu);
      last[1] = static_cast<uint8_t>((frame.tMs >> 8) & 0xFFu);
      return true;
    }
  }

  if (gCount >= control::kAutonMaxFrames) {
    return false;
  }
  std::memcpy(framePtr(gCount), &frame, sizeof(frame));
  ++gCount;
  return true;
}

uint16_t frameCount() {
  return gCount;
}

uint32_t byteCount() {
  return static_cast<uint32_t>(gCount) * control::kAutonFrameBytes;
}

bool save(const char* path) {
  if (path == 0 || gCount == 0) {
    return false;
  }
  const uint32_t payload = byteCount();
  if (!control::autonWriteHeader(gBuffer, sizeof(gBuffer), payload, gCount,
                                 kPeriodMs)) {
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
