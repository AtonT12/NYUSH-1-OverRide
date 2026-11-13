#pragma once

namespace SecondaryLift {

bool isReady();

// Positive power raises the articulated lift.
void setPower(double percent);
void stop();

}  // namespace SecondaryLift
