#include "control/mechanism.h"

#include <cmath>

namespace control {
namespace {

double clampRange(double value, double low, double high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

}  // namespace

double clampToLimits(const AxisCal& cal, double targetDeg, bool* clamped) {
  const double low = cal.minDeg <= cal.maxDeg ? cal.minDeg : cal.maxDeg;
  const double high = cal.minDeg <= cal.maxDeg ? cal.maxDeg : cal.minDeg;
  const double limited = clampRange(targetDeg, low, high);
  if (clamped != 0) {
    *clamped = limited != targetDeg;
  }
  return limited;
}

double limitRate(double fromDeg, double toDeg, double maxStepDeg) {
  if (maxStepDeg <= 0.0) {
    return fromDeg;
  }
  const double delta = toDeg - fromDeg;
  if (delta > maxStepDeg) {
    return fromDeg + maxStepDeg;
  }
  if (delta < -maxStepDeg) {
    return fromDeg - maxStepDeg;
  }
  return toDeg;
}

bool inBand(double actualDeg, double targetDeg, double bandDeg) {
  const double band = bandDeg < 0.0 ? -bandDeg : bandDeg;
  return std::fabs(actualDeg - targetDeg) <= band;
}

double approachSpeedScale(const AxisCal& cal, double remainingDeg) {
  const double approach = cal.approachDeg < 0.0 ? -cal.approachDeg : cal.approachDeg;
  const double remaining = std::fabs(remainingDeg);
  if (approach > 0.0 && remaining <= approach) {
    return 0.25;
  }
  return 1.0;
}

bool shouldStopHoming(double currentAmp, double posDeltaDeg, uint16_t stableTicks,
                      const AxisCal& cal) {
  if (stableTicks < cal.homingStableTicks) {
    return false;
  }
  if (currentAmp < cal.homingCurrentAmp) {
    return false;
  }
  return std::fabs(posDeltaDeg) <= cal.toleranceDeg;
}

}  // namespace control
