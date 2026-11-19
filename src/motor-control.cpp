#include "motor-control.h"

#include "control/shaping.h"

#include <cmath>

namespace MotorControl {

int applyDeadband(int value, int deadband) {
  // One definition of the deadband, shared with teleop and playback. The
  // arithmetic lives in the control core so it can be covered by host tests;
  // this wrapper exists only for callers that still work in controller units.
  return control::applyDeadbandPercent(value, deadband);
}

double limitMagnitude(double value, double maxMagnitude) {
  if (maxMagnitude < 0.0) {
    maxMagnitude = 0.0;
  }

  if (value > maxMagnitude) {
    return maxMagnitude;
  }
  if (value < -maxMagnitude) {
    return -maxMagnitude;
  }

  return value;
}

void spinRpm(vex::motor* motor, double rpm) {
  if (motor == nullptr) {
    return;
  }

  if (std::fabs(rpm) < 0.001) {
    motor->stop(vex::brakeType::brake);
    return;
  }

  if (rpm > 0.0) {
    motor->spin(vex::directionType::fwd, rpm, vex::velocityUnits::rpm);
  } else {
    motor->spin(vex::directionType::rev, -rpm, vex::velocityUnits::rpm);
  }
}

void stopMotor(vex::motor* motor, vex::brakeType mode) {
  if (motor != nullptr) {
    motor->stop(mode);
  }
}

}  // namespace MotorControl
