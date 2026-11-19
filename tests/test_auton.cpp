#include "control/auton.h"
#include "harness.h"

namespace {

const control::AutonFrame kFrames[] = {
    {0u, 0, 0, 0, 0, 0u, 0u},
    {100u, 1000, -500, 0, 250, control::kFlagToggle, 0u},
};

const control::AutonProgram kProgram(kFrames, 2u, 20u);

const control::AutonProgram kEmpty;

}  // namespace

TEST_CASE(auton_frame_is_twelve_bytes) {
  // The wire format is shared with the Python encoder; a change here is a
  // breaking change for every recorded file.
  CHECK_INT(sizeof(control::AutonFrame), 12u);
}

TEST_CASE(auton_empty_program_is_a_no_op) {
  const control::UserCommand c = control::sampleAuton(kEmpty, 500u);
  CHECK_NEAR(c.forward, 0.0, 1e-12);
  CHECK_NEAR(c.turn, 0.0, 1e-12);
  CHECK_INT(control::autonDurationMs(kEmpty), 0u);
}

TEST_CASE(auton_clamps_before_the_first_frame) {
  const control::UserCommand c = control::sampleAuton(kProgram, 0u);
  CHECK_NEAR(c.forward, 0.0, 1e-12);
  CHECK_INT(c.toggle, 0);
}

TEST_CASE(auton_interpolates_the_continuous_channels) {
  const control::UserCommand c = control::sampleAuton(kProgram, 50u);
  CHECK_NEAR(c.forward, 0.5, 1e-12);
  CHECK_NEAR(c.strafe, -0.25, 1e-12);
  CHECK_NEAR(c.liftCoarse, 0.125, 1e-12);
}

TEST_CASE(auton_holds_discrete_channels_until_the_next_frame) {
  const control::AutonFrame frames[] = {
      {0u, 0, 0, 0, 0, 0u, 0u},
      {100u, 0, 0, 0, 0, control::kFlagToggle, 0u},
      {200u, 0, 0, 0, 0, 0u, 0u},
  };
  const control::AutonProgram program(frames, 3u, 20u);

  // Before the flag frame the channel is still released.
  CHECK_FALSE(control::sampleAuton(program, 50u).toggle);
  // Held for the whole interval that starts at the flag frame.
  CHECK_TRUE(control::sampleAuton(program, 150u).toggle);
  // Released again by the following frame.
  CHECK_FALSE(control::sampleAuton(program, 200u).toggle);
}

TEST_CASE(auton_clamps_after_the_last_frame) {
  const control::UserCommand c = control::sampleAuton(kProgram, 5000u);
  CHECK_NEAR(c.forward, 1.0, 1e-12);
  CHECK_TRUE(c.toggle);
}

TEST_CASE(auton_lands_exactly_on_the_last_frame) {
  const control::UserCommand c = control::sampleAuton(kProgram, 100u);
  CHECK_NEAR(c.forward, 1.0, 1e-12);
  CHECK_NEAR(c.strafe, -0.5, 1e-12);
  CHECK_NEAR(c.liftCoarse, 0.25, 1e-12);
}

TEST_CASE(auton_duration_is_the_last_timestamp) {
  CHECK_INT(control::autonDurationMs(kProgram), 100u);
}

TEST_CASE(auton_flags_map_to_discrete_commands) {
  using namespace control;
  const AutonFrame one[] = {
      {0u, 0, 0, 0, 0,
       static_cast<uint8_t>(kFlagLiftFinePositive | kFlagFixtureNegative), 0u},
  };
  const AutonProgram program(one, 1u, 20u);
  const UserCommand c = sampleAuton(program, 0u);
  CHECK_INT(c.liftFine, 1);
  CHECK_INT(c.fixture, -1);
  CHECK_INT(c.toggle, 0);
}

TEST_CASE(permille_encoding_rounds_and_clamps) {
  CHECK_INT(control::autonToPermille(0.5), 500);
  CHECK_INT(control::autonToPermille(-0.5), -500);
  CHECK_INT(control::autonToPermille(1.5), 1000);
  CHECK_INT(control::autonToPermille(-1.5), -1000);
  CHECK_INT(control::autonToPermille(0.0004), 0);
  CHECK_INT(control::autonToPermille(0.0005), 1);
  CHECK_INT(control::autonToPermille(-0.0005), -1);
}

TEST_CASE(crc32_matches_the_standard_check_vector) {
  const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_INT(control::autonCrc32(text, sizeof(text)), 0xCBF43926u);
}

TEST_CASE(encode_frame_carries_values_and_flags) {
  control::UserCommand c;
  c.forward = 0.5;
  c.strafe = -0.25;
  c.turn = 1.0;
  c.liftCoarse = -1.0;
  c.liftFine = -1;
  c.fixture = 1;
  c.toggle = true;

  control::AutonFrame frame;
  control::autonEncodeFrame(c, 1234u, &frame);

  CHECK_INT(frame.tMs, 1234);
  CHECK_INT(frame.forwardPermille, 500);
  CHECK_INT(frame.strafePermille, -250);
  CHECK_INT(frame.turnPermille, 1000);
  CHECK_INT(frame.liftCoarsePermille, -1000);
  CHECK_TRUE((frame.flags & control::kFlagLiftFineNegative) != 0);
  CHECK_TRUE((frame.flags & control::kFlagFixturePositive) != 0);
  CHECK_TRUE((frame.flags & control::kFlagToggle) != 0);
  CHECK_INT(frame.reserved, 0);
}

TEST_CASE(header_writes_the_documented_layout) {
  // The recorder encodes frames in place and then fills in the header, so this
  // is the only place the on-disk layout is pinned down on the firmware side.
  uint8_t buffer[control::kAutonHeaderBytes + 24] = {0};
  CHECK_TRUE(control::autonWriteHeader(buffer, sizeof(buffer), 24u, 2u, 20u));

  CHECK_INT(buffer[0], static_cast<unsigned>('O'));
  CHECK_INT(buffer[1], static_cast<unsigned>('V'));
  CHECK_INT(buffer[2], static_cast<unsigned>('R'));
  CHECK_INT(buffer[3], static_cast<unsigned>('R'));
  CHECK_INT(buffer[4], 1u);
  CHECK_INT(buffer[6] | (buffer[7] << 8), 20u);
  CHECK_INT(buffer[8] | (buffer[9] << 8), 2u);
}

TEST_CASE(header_refuses_a_payload_that_does_not_fit) {
  uint8_t buffer[16];
  CHECK_FALSE(control::autonWriteHeader(buffer, sizeof(buffer), 24u, 2u, 20u));
}