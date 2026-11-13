#include "vertical-lift.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace VerticalLift {

bool isReady() {
  return RobotHardware::primaryLiftReady();
}

void setPower(double percent) {
  if (!isReady()) {
    return;
  }

  const double limited =
      MotorControl::limitMagnitude(percent, RobotConstants::kMechanismMaxPercent);
  MotorControl::spinPercent(RobotHardware::primaryLift.motorA, limited);
  MotorControl::spinPercent(RobotHardware::primaryLift.motorB, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::primaryLift.motorA,
                          vex::brakeType::brake);
  MotorControl::stopMotor(RobotHardware::primaryLift.motorB,
                          vex::brakeType::brake);
}

}  // namespace VerticalLift
