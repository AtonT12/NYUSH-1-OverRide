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

TEST_CASE(serialize_round_trips_through_deserialize) {
  const control::AutonFrame frames[] = {
      {0u, 0, 0, 0, 0, 0u, 0u},
      {500u, 1000, -500, 250, 0, control::kFlagToggle, 0u},
      {1000u, 0, 0, 0, 0, 0u, 0u},
  };
  const control::AutonProgram source(frames, 3u, 20u);

  uint8_t buffer[128];
  const uint32_t written = control::autonSerialize(source, buffer, sizeof(buffer));
  CHECK_INT(written, control::kAutonHeaderBytes + 3u * control::kAutonFrameBytes);

  control::AutonFrame decoded[8];
  control::AutonProgram restored;
  CHECK_TRUE(control::autonDeserialize(buffer, written, decoded, 8u, &restored));
  CHECK_INT(restored.count, 3);
  CHECK_INT(restored.periodMs, 20);
  CHECK_INT(restored.frames[1].tMs, 500);
  CHECK_INT(restored.frames[1].forwardPermille, 1000);
  CHECK_INT(restored.frames[1].strafePermille, -500);
  CHECK_INT(restored.frames[1].turnPermille, 250);
  CHECK_TRUE((restored.frames[1].flags & control::kFlagToggle) != 0);
}

TEST_CASE(serialize_refuses_a_buffer_that_is_too_small) {
  const control::AutonFrame frames[] = {{0u, 0, 0, 0, 0, 0u, 0u}};
  const control::AutonProgram source(frames, 1u, 20u);
  uint8_t buffer[16];
  CHECK_INT(control::autonSerialize(source, buffer, sizeof(buffer)), 0u);
}

TEST_CASE(deserialize_rejects_bad_magic) {
  const control::AutonFrame frames[] = {{0u, 0, 0, 0, 0, 0u, 0u}};
  const control::AutonProgram source(frames, 1u, 20u);
  uint8_t buffer[64];
  uint32_t written = control::autonSerialize(source, buffer, sizeof(buffer));
  buffer[0] = 'X';

  control::AutonFrame decoded[4];
  control::AutonProgram restored;
  CHECK_FALSE(control::autonDeserialize(buffer, written, decoded, 4u, &restored));
  CHECK_INT(restored.count, 0);
}

TEST_CASE(deserialize_rejects_a_corrupted_payload) {
  const control::AutonFrame frames[] = {
      {0u, 0, 0, 0, 0, 0u, 0u},
      {100u, 500, 0, 0, 0, 0u, 0u},
  };
  const control::AutonProgram source(frames, 2u, 20u);
  uint8_t buffer[64];
  const uint32_t written = control::autonSerialize(source, buffer, sizeof(buffer));
  buffer[control::kAutonHeaderBytes + 2] ^= 0xFFu;  // flip a payload byte

  control::AutonFrame decoded[4];
  control::AutonProgram restored;
  CHECK_FALSE(control::autonDeserialize(buffer, written, decoded, 4u, &restored));
}

TEST_CASE(deserialize_rejects_truncated_data) {
  const control::AutonFrame frames[] = {
      {0u, 0, 0, 0, 0, 0u, 0u},
      {100u, 500, 0, 0, 0, 0u, 0u},
  };
  const control::AutonProgram source(frames, 2u, 20u);
  uint8_t buffer[64];
  const uint32_t written = control::autonSerialize(source, buffer, sizeof(buffer));

  control::AutonFrame decoded[4];
  control::AutonProgram restored;
  CHECK_FALSE(control::autonDeserialize(buffer, written - 1u, decoded, 4u, &restored));
}
