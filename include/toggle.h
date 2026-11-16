#pragma once

namespace Toggle {

bool isReady();

// Positive RPM rotates the Toggle upward; negative RPM rotates it downward.
void setRpm(double rpm);
void stop();

}  // namespace Toggle
