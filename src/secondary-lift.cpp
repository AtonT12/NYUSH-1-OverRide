#include "secondary-lift.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace SecondaryLift {

bool isReady() {
  return RobotHardware::secondaryLiftReady();
}

void setRpm(double rpm) {
  if (!isReady()) {
    stop();
    return;
  }

  const double limited = MotorControl::limitMagnitude(
      rpm, RobotConstants::kMechanismMaxCommandRpm);
  MotorControl::spinRpm(RobotHardware::secondaryLiftMotor, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::secondaryLiftMotor,
                          vex::brakeType::brake);
}

}  // namespace SecondaryLift
