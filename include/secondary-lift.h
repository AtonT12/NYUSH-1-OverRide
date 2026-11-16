#pragma once

namespace SecondaryLift {

bool isReady();

// Positive RPM raises the articulated lift.
void setRpm(double rpm);
void stop();

}  // namespace SecondaryLift
