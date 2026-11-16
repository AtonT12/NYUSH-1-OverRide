#include "motor-control.h"

#include <cmath>

namespace MotorControl {

int applyDeadband(int value, int deadband) {
  if (deadband < 0) {
    deadband = 0;
  }
  if (deadband > 99) {
    deadband = 99;
  }

  value = static_cast<int>(limitMagnitude(value, 100.0));
  const int magnitude = value < 0 ? -value : value;
  if (magnitude <= deadband) {
    return 0;
  }

  const int scaled = ((magnitude - deadband) * 100) / (100 - deadband);
  return value < 0 ? -scaled : scaled;
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
