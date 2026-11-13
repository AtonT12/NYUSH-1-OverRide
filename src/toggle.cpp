#include "toggle.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Toggle {

bool isReady() {
  return RobotHardware::toggleReady();
}

void setPower(double percent) {
  if (!isReady()) {
    return;
  }

  const double limited =
      MotorControl::limitMagnitude(percent, RobotConstants::kMechanismMaxPercent);
  MotorControl::spinPercent(RobotHardware::toggleMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::toggleMotor, vex::brakeType::brake);
}

}  // namespace Toggle
