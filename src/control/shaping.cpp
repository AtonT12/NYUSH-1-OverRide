#include "control/shaping.h"

#include <cmath>

namespace control {
namespace {

double clampUnit(double value) {
  if (value > 1.0) {
    return 1.0;
  }
  if (value < -1.0) {
    return -1.0;
  }
  return value;
}

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

double applyDeadbandUnit(double value, double deadband) {
  const double v = clampUnit(value);
  // 1.0 would divide by zero; anything above 0.99 makes the stick unusable.
  const double db = clampRange(deadband, 0.0, 0.99);
  const double magnitude = std::fabs(v);
  if (magnitude <= db) {
    return 0.0;
  }
  // Rescale the remaining travel back to full range so the operator keeps the
  // whole output span instead of losing the first `db` of it.
  const double scaled = (magnitude - db) / (1.0 - db);
  return v < 0.0 ? -scaled : scaled;
}

int applyDeadbandPercent(int value, int deadband) {
  const double shaped =
      applyDeadbandUnit(static_cast<double>(value) / 100.0,
                        static_cast<double>(deadband) / 100.0);
  const double scaled = shaped * 100.0;
  // Half-away-from-zero rounding keeps left and right symmetric.
  return static_cast<int>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

double slewScalar(double current, double target, double maxDelta) {
  if (maxDelta <= 0.0) {
    return current;
  }
  const double delta = target - current;
  if (delta > maxDelta) {
    return current + maxDelta;
  }
  if (delta < -maxDelta) {
    return current - maxDelta;
  }
  return target;
}

void slewVector(double* x, double* y, double targetX, double targetY, double maxDelta) {
  if (x == 0 || y == 0 || maxDelta <= 0.0) {
    return;
  }
  const double dx = targetX - *x;
  const double dy = targetY - *y;
  const double length = std::sqrt(dx * dx + dy * dy);
  if (length <= maxDelta || length <= 1e-9) {
    *x = targetX;
    *y = targetY;
    return;
  }
  const double scale = maxDelta / length;
  *x += dx * scale;
  *y += dy * scale;
}

UserCommand shape(const UserCommand& raw, SlewState* state, const ShapingCal& cal,
                  double dtSec) {
  UserCommand out = raw;
  out.forward = applyDeadbandUnit(raw.forward, cal.translationDeadband);
  out.strafe = applyDeadbandUnit(raw.strafe, cal.translationDeadband);
  out.turn = applyDeadbandUnit(raw.turn, cal.turnDeadband);
  out.liftCoarse = applyDeadbandUnit(raw.liftCoarse, cal.liftCoarseDeadband);

  if (state == 0) {
    return out;
  }

  if (dtSec > 0.0) {
    // A rate of zero means the channel is not shaped at all, which is how the
    // reference treats the coarse lift: only translation and turn ramp.
    if (cal.translationSlewPerSec > 0.0) {
      slewVector(&state->forward, &state->strafe, out.forward, out.strafe,
                 cal.translationSlewPerSec * dtSec);
    } else {
      state->forward = out.forward;
      state->strafe = out.strafe;
    }

    if (cal.turnSlewPerSec > 0.0) {
      state->turn = slewScalar(state->turn, out.turn, cal.turnSlewPerSec * dtSec);
    } else {
      state->turn = out.turn;
    }

    if (cal.liftCoarseSlewPerSec > 0.0) {
      state->liftCoarse = slewScalar(state->liftCoarse, out.liftCoarse,
                                     cal.liftCoarseSlewPerSec * dtSec);
    } else {
      state->liftCoarse = out.liftCoarse;
    }
  }

  out.forward = state->forward;
  out.strafe = state->strafe;
  out.turn = state->turn;
  out.liftCoarse = state->liftCoarse;
  return out;
}

}  // namespace control
