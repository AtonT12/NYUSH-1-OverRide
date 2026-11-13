#include "motor-control.h"

#include <cmath>

namespace MotorControl {

int applyDeadband(int value, int deadband) {
  if (deadband < 0) {
    deadband = 0;
  }

  if (value > deadband || value < -deadband) {
    return value;
  }

  return 0;
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

void spinPercent(vex::motor* motor, double percent) {
  if (motor == nullptr) {
    return;
  }

  percent = limitMagnitude(percent, 100.0);

  if (std::fabs(percent) < 0.001) {
    motor->stop(vex::brakeType::brake);
    return;
  }

  if (percent > 0.0) {
    motor->spin(vex::directionType::fwd, percent, vex::percentUnits::pct);
  } else {
    motor->spin(vex::directionType::rev, -percent, vex::percentUnits::pct);
  }
}

void stopMotor(vex::motor* motor, vex::brakeType mode) {
  if (motor != nullptr) {
    motor->stop(mode);
  }
}

}  // namespace MotorControl
