#include "control/kinematics.h"
#include "harness.h"

namespace {

const control::DriveCal kCal = {500.0, 200.0, 480.0};

const double kForwardOnly = 353.55339059327378;  // 500 / sqrt(2)
const double kDiagonalPeak = 907.10678118654757; // 500 + 500/sqrt(2) + 200

control::UserCommand command(double forward, double strafe, double turn) {
  control::UserCommand c;
  c.forward = forward;
  c.strafe = strafe;
  c.turn = turn;
  return c;
}

}  // namespace

TEST_CASE(mix_forward_pushes_all_four_wheels_equally) {
  const control::WheelRpm w = control::mixXDrive(command(1.0, 0.0, 0.0), kCal);
  CHECK_NEAR(w.fl, kForwardOnly, 1e-9);
  CHECK_NEAR(w.fr, kForwardOnly, 1e-9);
  CHECK_NEAR(w.rl, kForwardOnly, 1e-9);
  CHECK_NEAR(w.rr, kForwardOnly, 1e-9);
}

TEST_CASE(mix_strafe_is_antisymmetric_across_diagonals) {
  const control::WheelRpm w = control::mixXDrive(command(0.0, 1.0, 0.0), kCal);
  CHECK_NEAR(w.fl, kForwardOnly, 1e-9);
  CHECK_NEAR(w.fr, -kForwardOnly, 1e-9);
  CHECK_NEAR(w.rl, -kForwardOnly, 1e-9);
  CHECK_NEAR(w.rr, kForwardOnly, 1e-9);
}

TEST_CASE(mix_turn_spins_diagonals_against_each_other) {
  const control::WheelRpm w = control::mixXDrive(command(0.0, 0.0, 1.0), kCal);
  CHECK_NEAR(w.fl, 200.0, 1e-9);
  CHECK_NEAR(w.fr, -200.0, 1e-9);
  CHECK_NEAR(w.rl, 200.0, 1e-9);
  CHECK_NEAR(w.rr, -200.0, 1e-9);
}

TEST_CASE(mix_zero_command_is_zero) {
  const control::WheelRpm w = control::mixXDrive(command(0.0, 0.0, 0.0), kCal);
  CHECK_NEAR(w.fl, 0.0, 1e-12);
  CHECK_NEAR(w.fr, 0.0, 1e-12);
  CHECK_NEAR(w.rl, 0.0, 1e-12);
  CHECK_NEAR(w.rr, 0.0, 1e-12);
}

TEST_CASE(clamp_is_a_noop_below_the_cap) {
  control::WheelRpm w = control::mixXDrive(command(1.0, 0.0, 0.0), kCal);
  const control::WheelRpm clamped = control::clampWheels(w, 480.0);
  CHECK_NEAR(clamped.fl, w.fl, 1e-12);
  CHECK_NEAR(clamped.rr, w.rr, 1e-12);
}

TEST_CASE(clamp_scales_the_whole_vector_to_the_cap) {
  const control::WheelRpm raw = control::mixXDrive(command(1.0, 1.0, 1.0), kCal);
  const control::WheelRpm w = control::clampWheels(raw, 480.0);

  CHECK_NEAR(control::wheelPeak(w), 480.0, 1e-9);
  CHECK_NEAR(w.fl, 480.0, 1e-9);

  // The ratio between wheels must survive the clamp; that is the whole reason
  // this is a proportional scale instead of four independent clips.
  CHECK_NEAR(w.fr / w.fl, -200.0 / kDiagonalPeak, 1e-9);
  CHECK_NEAR(w.rl / w.fl, 200.0 / kDiagonalPeak, 1e-9);
  CHECK_NEAR(w.rr / w.fl, 507.10678118654757 / kDiagonalPeak, 1e-9);
}

TEST_CASE(clamp_with_zero_cap_outputs_zero) {
  const control::WheelRpm w = control::clampWheels(
      control::mixXDrive(command(1.0, 0.0, 0.0), kCal), 0.0);
  CHECK_NEAR(w.fl, 0.0, 1e-12);
  CHECK_NEAR(w.rr, 0.0, 1e-12);
}

TEST_CASE(wheel_peak_reads_the_largest_magnitude) {
  control::WheelRpm w;
  w.fl = -12.0;
  w.fr = 3.0;
  w.rl = 7.0;
  w.rr = -11.0;
  CHECK_NEAR(control::wheelPeak(w), 12.0, 1e-12);
}
