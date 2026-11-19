#include "coarse-lift.h"

#include "constants.h"
#include "mechanism.h"
#include "robot-config.h"

namespace {

bool gWasNegative = false;
double gMinDeg = 0.0;

// Reference main.rs:268-269 and :322: 15 and 202 degrees, and the approach
// runs at LIFT_COARSE_SPEED * 0.25.
const double kDecelerationBandDeg = 15.0;
const double kStopLimitDeg = 202.0;
const double kEngageThreshold = 0.01;
const double kApproachSpeedFactor = 0.25;

}  // namespace

namespace CoarseLift {

void reset() {
  gWasNegative = false;
  gMinDeg = 0.0;
}

void apply(double amount, double maxRpm) {
  if (!RobotHardware::secondaryLiftReady()) {
    Mechanism::stop(RobotHardware::secondaryLift);
    return;
  }

  vex::motor* const motor = RobotHardware::secondaryLift.motorA;
  const double angle = motor->position(vex::degrees);

  // Track the bottom of travel while the axis is below zero. The first negative
  // sample replaces the power-on zero, which is what lets a match recover from
  // an arbitrary starting height.
  const bool negative = angle < 0.0;
  if (negative) {
    if (!gWasNegative || angle < gMinDeg) {
      gMinDeg = angle;
    }
  }
  gWasNegative = negative;

  const double target = gMinDeg + kStopLimitDeg;
  if (amount > kEngageThreshold && angle > target - kDecelerationBandDeg) {
    // Hand over to the position loop instead of pushing into the stop.
    // Non-blocking, so the control period is unaffected.
    motor->spinToPosition(target, vex::degrees, maxRpm * kApproachSpeedFactor,
                          vex::velocityUnits::rpm, false);
    return;
  }

  Mechanism::setRpm(RobotHardware::secondaryLift, amount * maxRpm,
                    RobotConstants::kMechanismMaxCommandRpm);
}

}  // namespace CoarseLift
