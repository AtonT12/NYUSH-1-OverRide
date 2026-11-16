#include "claw.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Claw {

bool isReady() {
  return RobotHardware::clawReady();
}

void setRpm(double rpm) {
  if (!isReady()) {
    stop();
    return;
  }

  const double limited = MotorControl::limitMagnitude(
      rpm, RobotConstants::kMechanismMaxCommandRpm);
  MotorControl::spinRpm(RobotHardware::clawMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::clawMotor, vex::brakeType::brake);
}

}  // namespace Claw
