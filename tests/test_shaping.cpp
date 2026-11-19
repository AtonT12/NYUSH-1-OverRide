#include "control/shaping.h"
#include "harness.h"

namespace {

const control::ShapingCal kCal = {0.0, 0.20, 0.20, 4.0, 2.0, 0.0};

control::UserCommand command(double forward, double strafe, double turn,
                             double liftCoarse) {
  control::UserCommand c;
  c.forward = forward;
  c.strafe = strafe;
  c.turn = turn;
  c.liftCoarse = liftCoarse;
  return c;
}

}  // namespace

TEST_CASE(deadband_zeroes_the_inner_region) {
  CHECK_NEAR(control::applyDeadbandUnit(0.0, 0.20), 0.0, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(0.199, 0.20), 0.0, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(0.20, 0.20), 0.0, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(-0.20, 0.20), 0.0, 1e-12);
}

TEST_CASE(deadband_rescales_the_remaining_travel) {
  CHECK_NEAR(control::applyDeadbandUnit(0.6, 0.20), 0.5, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(-0.6, 0.20), -0.5, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(1.0, 0.20), 1.0, 1e-12);
}

TEST_CASE(deadband_clamps_out_of_range_input) {
  CHECK_NEAR(control::applyDeadbandUnit(2.5, 0.20), 1.0, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(-2.5, 0.20), -1.0, 1e-12);
}

TEST_CASE(deadband_of_zero_is_the_identity) {
  CHECK_NEAR(control::applyDeadbandUnit(0.37, 0.0), 0.37, 1e-12);
  CHECK_NEAR(control::applyDeadbandUnit(-0.01, 0.0), -0.01, 1e-12);
}

TEST_CASE(slew_scalar_limits_the_step) {
  CHECK_NEAR(control::slewScalar(0.0, 1.0, 0.08), 0.08, 1e-12);
  CHECK_NEAR(control::slewScalar(0.0, -1.0, 0.08), -0.08, 1e-12);
}

TEST_CASE(slew_scalar_snaps_when_within_reach) {
  CHECK_NEAR(control::slewScalar(0.95, 1.0, 0.08), 1.0, 1e-12);
  CHECK_NEAR(control::slewScalar(0.0, 0.05, 0.08), 0.05, 1e-12);
}

TEST_CASE(slew_vector_limits_magnitude_and_keeps_direction) {
  double x = 0.0;
  double y = 0.0;
  control::slewVector(&x, &y, 1.0, 1.0, 0.10);
  CHECK_NEAR(x, y, 1e-12);
  CHECK_NEAR(x * x + y * y, 0.01, 1e-12);
}

TEST_CASE(shape_keeps_translation_undeadbanded) {
  control::SlewState state;
  const control::UserCommand out =
      control::shape(command(0.01, 0.0, 0.0, 0.0), &state, kCal, 0.02);
  CHECK_TRUE(out.forward > 0.0);
}

TEST_CASE(shape_applies_turn_deadband) {
  control::SlewState state;
  const control::UserCommand quiet =
      control::shape(command(0.0, 0.0, 0.19, 0.0), &state, kCal, 0.02);
  CHECK_NEAR(quiet.turn, 0.0, 1e-12);

  const control::UserCommand moving =
      control::shape(command(0.0, 0.0, 0.60, 0.0), &state, kCal, 0.02);
  CHECK_NEAR(moving.turn, 0.04, 1e-12);  // 0.5 rescaled, then one 2.0/s step
}

TEST_CASE(shape_applies_lift_coarse_deadband) {
  control::SlewState state;
  const control::UserCommand out =
      control::shape(command(0.0, 0.0, 0.0, 0.19), &state, kCal, 0.02);
  CHECK_NEAR(out.liftCoarse, 0.0, 1e-12);
}

TEST_CASE(shape_leaves_lift_coarse_unshaped) {
  // A zero rate means "no limiter": the coarse lift follows the stick on the
  // same tick, matching the reference.
  control::SlewState state;
  const control::UserCommand out =
      control::shape(command(0.0, 0.0, 0.0, 1.0), &state, kCal, 0.02);
  CHECK_NEAR(out.liftCoarse, 1.0, 1e-12);
}

TEST_CASE(shape_reaches_full_stick_in_thirteen_ticks) {
  // 4/s over a 20 ms tick is 0.08 per tick, so a full push takes 13 ticks
  // (250 ms). The reference reaches full stick in 10 frames at 25 ms (250 ms).
  control::SlewState state;
  control::UserCommand out;
  for (int i = 0; i < 12; ++i) {
    out = control::shape(command(1.0, 0.0, 0.0, 0.0), &state, kCal, 0.02);
  }
  CHECK_NEAR(out.forward, 0.96, 1e-12);
  out = control::shape(command(1.0, 0.0, 0.0, 0.0), &state, kCal, 0.02);
  CHECK_NEAR(out.forward, 1.0, 1e-12);
}

TEST_CASE(shape_freezes_when_dt_is_not_positive) {
  control::SlewState state;
  const control::UserCommand out =
      control::shape(command(1.0, 0.0, 0.0, 0.0), &state, kCal, 0.0);
  CHECK_NEAR(out.forward, 0.0, 1e-12);
}

TEST_CASE(shape_without_state_still_shapes_deadband) {
  const control::UserCommand out =
      control::shape(command(0.0, 0.0, 0.19, 0.0), 0, kCal, 0.02);
  CHECK_NEAR(out.turn, 0.0, 1e-12);
}
