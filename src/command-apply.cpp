#include "command-apply.h"

#include "claw.h"
#include "constants.h"
#include "drivetrain.h"
#include "secondary-lift.h"
#include "toggle.h"
#include "vertical-lift.h"

namespace CommandApply {

void apply(const control::UserCommand& command) {
  Drivetrain::apply(command);

  if (command.liftFine > 0) {
    VerticalLift::setRpm(RobotConstants::kPrimaryLiftRpm);
  } else if (command.liftFine < 0) {
    VerticalLift::setRpm(-RobotConstants::kPrimaryLiftRpm);
  } else {
    VerticalLift::stop();
  }

  if (command.fixture > 0) {
    Claw::setRpm(RobotConstants::kClawRpm);
  } else if (command.fixture < 0) {
    Claw::setRpm(-RobotConstants::kClawRpm);
  } else {
    Claw::stop();
  }

  if (command.toggle) {
    Toggle::setRpm(RobotConstants::kToggleRpm);
  } else {
    Toggle::stop();
  }

  SecondaryLift::setRpm(command.liftCoarse * RobotConstants::kSecondaryLiftRpm);
}

}  // namespace CommandApply
