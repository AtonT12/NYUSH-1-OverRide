#include "toggle.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Toggle {

bool isReady() {
  return RobotHardware::toggleReady();
}

void setRpm(double rpm) {
  if (!isReady()) {
    stop();
    return;
  }

  const double limited = MotorControl::limitMagnitude(
      rpm, RobotConstants::kMechanismMaxCommandRpm);
  MotorControl::spinRpm(RobotHardware::toggleMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::toggleMotor, vex::brakeType::brake);
}

}  // namespace Toggle
