#include "secondary-lift.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace SecondaryLift {

bool isReady() {
  return RobotHardware::secondaryLiftReady();
}

void setPower(double percent) {
  if (!isReady()) {
    return;
  }

  const double limited =
      MotorControl::limitMagnitude(percent, RobotConstants::kMechanismMaxPercent);
  MotorControl::spinPercent(RobotHardware::secondaryLiftMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::secondaryLiftMotor,
                          vex::brakeType::brake);
}

}  // namespace SecondaryLift
