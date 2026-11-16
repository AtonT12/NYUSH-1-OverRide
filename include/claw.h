#pragma once

namespace Claw {

bool isReady();

// Positive RPM closes the gripper; negative RPM opens it.
void setRpm(double rpm);
void stop();

}  // namespace Claw
