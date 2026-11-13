#pragma once

namespace Claw {

bool isReady();

// Positive power closes the gripper; negative power opens it.
void setPower(double percent);
void stop();

}  // namespace Claw
