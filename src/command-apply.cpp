#include "command-apply.h"

#include "coarse-lift.h"
#include "constants.h"
#include "drivetrain.h"
#include "mechanism.h"
#include "robot-config.h"

namespace {

// Reference main.rs:333-341: the fine lift refuses to drive below -20 degrees.
// The reference has no upper soft limit on this axis, so neither does this.
// Readiness is not checked here: Mechanism::setRpm() stops the axis if either
// motor is unbound, and position() on an unplugged port reads zero.
double fineLiftRpm(int8_t direction) {
  if (direction == 0) {
    return 0.0;
  }
  if (direction < 0 &&
      RobotHardware::primaryLift.motorA->position(vex::degrees) < -20.0) {
    return 0.0;
  }
  return direction * RobotConstants::kPrimaryLiftRpm;
}

}  // namespace

namespace CommandApply {

void apply(const control::UserCommand& command) {
  Drivetrain::apply(command);

  Mechanism::setRpm(RobotHardware::primaryLift, fineLiftRpm(command.liftFine),
                    RobotConstants::kMechanismMaxCommandRpm);

  Mechanism::setRpm(RobotHardware::claw,
                    command.fixture * RobotConstants::kClawRpm,
                    RobotConstants::kMechanismMaxCommandRpm);

  Mechanism::setRpm(RobotHardware::toggle,
                    command.toggle ? RobotConstants::kToggleRpm : 0.0,
                    RobotConstants::kMechanismMaxCommandRpm);

  CoarseLift::apply(command.liftCoarse, RobotConstants::kSecondaryLiftRpm);
}

}  // namespace CommandApply
