#pragma once

namespace Toggle {

bool isReady();

// Positive power rotates the Toggle upward; negative power rotates it downward.
void setPower(double percent);
void stop();

}  // namespace Toggle
