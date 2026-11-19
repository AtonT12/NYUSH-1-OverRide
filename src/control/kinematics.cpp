#include "control/kinematics.h"

#include <cmath>

namespace control {
namespace {

double absd(double value) {
  return value < 0.0 ? -value : value;
}

}  // namespace

WheelRpm mixXDrive(const UserCommand& command, const DriveCal& cal) {
  const double diagonal = std::sqrt(2.0);
  const double forward = command.forward * cal.translationRpm;
  const double strafe = command.strafe * cal.translationRpm;
  const double turn = command.turn * cal.turnRpm;

  WheelRpm wheels;
  wheels.fl = (strafe + forward) / diagonal + turn;
  wheels.fr = (-strafe + forward) / diagonal - turn;
  wheels.rl = (-strafe + forward) / diagonal + turn;
  wheels.rr = (strafe + forward) / diagonal - turn;
  return wheels;
}

double wheelPeak(const WheelRpm& wheels) {
  double peak = absd(wheels.fl);
  if (absd(wheels.fr) > peak) {
    peak = absd(wheels.fr);
  }
  if (absd(wheels.rl) > peak) {
    peak = absd(wheels.rl);
  }
  if (absd(wheels.rr) > peak) {
    peak = absd(wheels.rr);
  }
  return peak;
}

WheelRpm clampWheels(WheelRpm wheels, double capRpm) {
  if (capRpm <= 0.0) {
    return WheelRpm();
  }
  const double peak = wheelPeak(wheels);
  if (peak <= capRpm) {
    return wheels;
  }
  const double scale = capRpm / peak;
  wheels.fl *= scale;
  wheels.fr *= scale;
  wheels.rl *= scale;
  wheels.rr *= scale;
  return wheels;
}

}  // namespace control
