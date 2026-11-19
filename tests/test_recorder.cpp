#include "control/auton.h"
#include "harness.h"

#include <cstring>

namespace {

// Declared as AutonFrame rather than bytes so sampleAuton() reads real objects;
// the writer is still handed the same storage as a byte buffer.
alignas(4) control::AutonFrame gStorage[control::kAutonMaxFrames];
uint8_t* const kFrames = reinterpret_cast<uint8_t*>(gStorage);

control::UserCommand make(double forward, double strafe, double turn,
                          double liftCoarse, int8_t liftFine, int8_t fixture,
                          bool toggle) {
  control::UserCommand c;
  c.forward = forward;
  c.strafe = strafe;
  c.turn = turn;
  c.liftCoarse = liftCoarse;
  c.liftFine = liftFine;
  c.fixture = fixture;
  c.toggle = toggle;
  return c;
}

control::AutonFrame frameAt(const control::FrameWriter& writer, uint16_t index) {
  control::AutonFrame frame;
  std::memcpy(&frame, writer.base + index * control::kAutonFrameBytes,
              sizeof(frame));
  return frame;
}

// Appends one command per tick from tMs = from to to inclusive.
void recordSpan(control::FrameWriter* writer, const control::UserCommand& command,
                uint16_t from, uint16_t to) {
  for (uint16_t t = from; t <= to; t = static_cast<uint16_t>(t + 20u)) {
    control::frameWriterAppend(writer, command, t);
  }
}

}  // namespace

TEST_CASE(recorder_keeps_one_frame_per_tick) {
  // The bug this pins: identical consecutive commands used to be merged into
  // the previous frame, so a hold cost one frame placed at its end.
  control::FrameWriter writer(kFrames, control::kAutonMaxFrames);
  const control::UserCommand steady = make(0.5, 0.0, 0.0, 0.0, 0, 0, false);

  bool allAppended = true;
  for (uint16_t i = 0; i < 100; ++i) {
    allAppended = allAppended &&
                  control::frameWriterAppend(&writer, steady,
                                             static_cast<uint16_t>(i * 20u));
  }

  CHECK_TRUE(allAppended);
  CHECK_INT(writer.count, 100u);
  CHECK_INT(frameAt(writer, 50).tMs, 1000u);
  CHECK_INT(frameAt(writer, 99).tMs, 1980u);
  CHECK_INT(frameAt(writer, 50).forwardPermille, 500);
}

TEST_CASE(recorder_puts_a_discrete_press_on_the_tick_it_changed) {
  control::FrameWriter writer(kFrames, control::kAutonMaxFrames);
  const control::UserCommand released = make(0.0, 0.0, 0.0, 0.0, 0, 0, false);
  const control::UserCommand pressed = make(0.0, 0.0, 0.0, 0.0, 1, 0, false);

  recordSpan(&writer, released, 0u, 980u);
  recordSpan(&writer, pressed, 1000u, 2000u);
  control::frameWriterAppend(&writer, released, 2020u);

  CHECK_INT(writer.count, 102u);  // 50 + 51 + 1

  // The press must start at 1000, not at the end of the hold.
  bool foundPress = false;
  uint16_t pressIndex = 0;
  for (uint16_t i = 0; i < writer.count; ++i) {
    const control::AutonFrame frame = frameAt(writer, i);
    if ((frame.flags & control::kFlagLiftFinePositive) != 0) {
      foundPress = true;
      pressIndex = i;
      break;
    }
  }
  CHECK_TRUE(foundPress);
  CHECK_INT(frameAt(writer, pressIndex).tMs, 1000u);

  const control::AutonProgram program(
      reinterpret_cast<const control::AutonFrame*>(writer.base), writer.count,
      20u);
  // Halfway through the hold replay must still be pressing, not pulsing.
  CHECK_INT(control::sampleAuton(program, 1500u).liftFine, 1);
  CHECK_INT(control::sampleAuton(program, 1000u).liftFine, 1);
  // ... and the release at 2020 must end it.
  CHECK_INT(control::sampleAuton(program, 2020u).liftFine, 0);
}

TEST_CASE(recorder_does_not_turn_a_hold_into_a_ramp) {
  control::FrameWriter writer(kFrames, control::kAutonMaxFrames);
  const control::UserCommand idle = make(0.0, 0.0, 0.0, 0.0, 0, 0, false);
  const control::UserCommand full = make(1.0, 0.0, 0.0, 0.0, 0, 0, false);

  recordSpan(&writer, idle, 0u, 980u);
  recordSpan(&writer, full, 1000u, 2000u);

  const control::AutonProgram program(
      reinterpret_cast<const control::AutonFrame*>(writer.base), writer.count,
      20u);
  CHECK_NEAR(control::sampleAuton(program, 1000u).forward, 1.0, 1e-12);
  CHECK_NEAR(control::sampleAuton(program, 1500u).forward, 1.0, 1e-12);
  CHECK_NEAR(control::sampleAuton(program, 980u).forward, 0.0, 1e-12);
}

TEST_CASE(recorder_stops_at_the_frame_cap) {
  control::FrameWriter writer(kFrames, control::kAutonMaxFrames);
  const control::UserCommand steady = make(0.0, 0.0, 0.0, 0.0, 0, 0, false);

  bool allAppended = true;
  for (uint16_t i = 0; i < control::kAutonMaxFrames; ++i) {
    allAppended = allAppended &&
                  control::frameWriterAppend(&writer, steady,
                                             static_cast<uint16_t>(i * 20u));
  }
  CHECK_TRUE(allAppended);
  CHECK_INT(writer.count, 1500u);

  // 1500 frames at 20 ms already covers the 30 s period.
  CHECK_FALSE(control::frameWriterAppend(&writer, steady, 29980u));
  CHECK_INT(writer.count, 1500u);
}

TEST_CASE(recorder_rejects_a_timestamp_that_goes_backwards) {
  control::FrameWriter writer(kFrames, 4u);
  const control::UserCommand steady = make(0.0, 0.0, 0.0, 0.0, 0, 0, false);

  CHECK_TRUE(control::frameWriterAppend(&writer, steady, 100u));
  CHECK_TRUE(control::frameWriterAppend(&writer, steady, 120u));
  // The offline encoder refuses a non-monotonic table, so catch it here.
  CHECK_FALSE(control::frameWriterAppend(&writer, steady, 119u));
  CHECK_INT(writer.count, 2u);
  CHECK_TRUE(control::frameWriterAppend(&writer, steady, 120u));
}

TEST_CASE(recorder_writes_the_wire_format_verbatim) {
  control::FrameWriter writer(kFrames, 4u);
  const control::UserCommand mixed = make(0.25, -0.5, 0.75, -1.0, 1, -1, true);
  CHECK_TRUE(control::frameWriterAppend(&writer, mixed, 40u));

  control::AutonFrame expected;
  control::autonEncodeFrame(mixed, 40u, &expected);
  CHECK_INT(std::memcmp(&expected, writer.base, sizeof(expected)), 0);
}

TEST_CASE(recorder_capacity_zero_is_not_a_crash) {
  control::FrameWriter writer(kFrames, 0u);
  const control::UserCommand steady = make(0.0, 0.0, 0.0, 0.0, 0, 0, false);
  CHECK_FALSE(control::frameWriterAppend(&writer, steady, 0u));
  CHECK_INT(writer.count, 0u);
}
