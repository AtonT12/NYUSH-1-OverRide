#include "vertical-lift.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace VerticalLift {

bool isReady() {
  return RobotHardware::primaryLiftReady();
}

void setRpm(double rpm) {
  if (!isReady()) {
    // Never leave the surviving motor running on a half-bound pair.
    stop();
    return;
  }

  const double limited = MotorControl::limitMagnitude(
      rpm, RobotConstants::kMechanismMaxCommandRpm);
  MotorControl::spinRpm(RobotHardware::primaryLift.motorA, limited);
  MotorControl::spinRpm(RobotHardware::primaryLift.motorB, limited);
}

void stop() {
  MotorControl::stopMotor(RobotHardware::primaryLift.motorA,
                          vex::brakeType::brake);
  MotorControl::stopMotor(RobotHardware::primaryLift.motorB,
                          vex::brakeType::brake);
}

}  // namespace VerticalLift
