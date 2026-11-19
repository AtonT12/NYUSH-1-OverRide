#include "control/mechanism.h"
#include "harness.h"

namespace {

const control::AxisCal kLift = {0.0, 202.0, 2.0, 15.0, 45.0, 2.5, 10u, 8000u};

}  // namespace

TEST_CASE(limits_clamp_above_the_max) {
  bool clamped = false;
  const double target = control::clampToLimits(kLift, 250.0, &clamped);
  CHECK_NEAR(target, 202.0, 1e-12);
  CHECK_TRUE(clamped);
}

TEST_CASE(limits_clamp_below_the_min) {
  bool clamped = false;
  const double target = control::clampToLimits(kLift, -50.0, &clamped);
  CHECK_NEAR(target, 0.0, 1e-12);
  CHECK_TRUE(clamped);
}

TEST_CASE(limits_pass_through_inside_the_range) {
  bool clamped = true;
  const double target = control::clampToLimits(kLift, 90.0, &clamped);
  CHECK_NEAR(target, 90.0, 1e-12);
  CHECK_FALSE(clamped);
}

TEST_CASE(limits_tolerate_an_inverted_calibration) {
  control::AxisCal inverted = kLift;
  inverted.minDeg = 202.0;
  inverted.maxDeg = 0.0;
  bool clamped = false;
  CHECK_NEAR(control::clampToLimits(inverted, 300.0, &clamped), 202.0, 1e-12);
  CHECK_TRUE(clamped);
}

TEST_CASE(rate_limit_steps_towards_the_target) {
  CHECK_NEAR(control::limitRate(0.0, 100.0, 10.0), 10.0, 1e-12);
  CHECK_NEAR(control::limitRate(100.0, 0.0, 10.0), 90.0, 1e-12);
  CHECK_NEAR(control::limitRate(95.0, 100.0, 10.0), 100.0, 1e-12);
}

TEST_CASE(rate_limit_freezes_on_a_non_positive_step) {
  CHECK_NEAR(control::limitRate(42.0, 100.0, 0.0), 42.0, 1e-12);
}

TEST_CASE(band_check_is_inclusive) {
  CHECK_TRUE(control::inBand(90.0, 91.5, 2.0));
  CHECK_TRUE(control::inBand(90.0, 92.0, 2.0));
  CHECK_FALSE(control::inBand(90.0, 92.5, 2.0));
}

TEST_CASE(approach_window_slows_the_last_degrees) {
  CHECK_NEAR(control::approachSpeedScale(kLift, 10.0), 0.25, 1e-12);
  CHECK_NEAR(control::approachSpeedScale(kLift, 15.0), 0.25, 1e-12);
  CHECK_NEAR(control::approachSpeedScale(kLift, 16.0), 1.0, 1e-12);
  CHECK_NEAR(control::approachSpeedScale(kLift, -10.0), 0.25, 1e-12);
}

TEST_CASE(homing_needs_current_position_and_time_together) {
  // Stall current reached, but the axis is still moving: keep going.
  CHECK_FALSE(control::shouldStopHoming(3.0, 5.0, 20u, kLift));
  // Not enough stable ticks yet.
  CHECK_FALSE(control::shouldStopHoming(3.0, 0.5, 5u, kLift));
  // Current spike is absent.
  CHECK_FALSE(control::shouldStopHoming(1.0, 0.5, 20u, kLift));
  // All three conditions met.
  CHECK_TRUE(control::shouldStopHoming(3.0, 0.5, 20u, kLift));
}
