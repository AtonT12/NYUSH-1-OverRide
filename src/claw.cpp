#include "claw.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Claw {

bool isReady() {
  return RobotHardware::clawReady();
}

void setPower(double percent) {
  if (!isReady()) {
    return;
  }

  const double limited =
      MotorControl::limitMagnitude(percent, RobotConstants::kMechanismMaxPercent);
  MotorControl::spinPercent(RobotHardware::clawMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::clawMotor, vex::brakeType::brake);
}

}  // namespace Claw
