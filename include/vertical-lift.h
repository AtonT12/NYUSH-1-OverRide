#pragma once

namespace VerticalLift {

bool isReady();

// Positive RPM raises the assembly. Both motors receive the same logical
// command; motor B is reversed in robot-config.cpp.
void setRpm(double rpm);
void stop();

}  // namespace VerticalLift
