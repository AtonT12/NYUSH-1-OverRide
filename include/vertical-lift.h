#pragma once

namespace VerticalLift {

bool isReady();

// Positive power raises the assembly. Both motors receive the same logical
// command; motor B is reverse-normalized in robot-config.cpp.
void setPower(double percent);
void stop();

}  // namespace VerticalLift
