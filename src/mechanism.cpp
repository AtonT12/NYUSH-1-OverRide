#include "mechanism.h"

#include "motor-control.h"

#include <cmath>

namespace {

void spinOne(vex::motor* motor, double rpm) {
  if (motor == nullptr) {
    return;
  }
  if (rpm > 0.0) {
    motor->spin(vex::directionType::fwd, rpm, vex::velocityUnits::rpm);
  } else {
    motor->spin(vex::directionType::rev, -rpm, vex::velocityUnits::rpm);
  }
}

}  // namespace

namespace Mechanism {

bool isReady(const RobotHardware::MotorPair& axis) {
  return RobotHardware::axisReady(axis);
}

void setRpm(const RobotHardware::MotorPair& axis, double rpm, double maxRpm) {
  if (!isReady(axis)) {
    // Never leave the surviving motor of a half-bound pair running.
    stop(axis);
    return;
  }

  const double limited = MotorControl::limitMagnitude(rpm, maxRpm);
  if (std::fabs(limited) < 0.001) {
    stop(axis);
    return;
  }

  spinOne(axis.motorA, limited);
  spinOne(axis.motorB, limited);
}

void stop(const RobotHardware::MotorPair& axis) {
  // No explicit brake mode: motor::stop() applies setStopping(), which is hold
  // for every mechanism axis. Passing brake here would drop a held object.
  if (axis.motorA != nullptr) {
    axis.motorA->stop();
  }
  if (axis.motorB != nullptr) {
    axis.motorB->stop();
  }
}

}  // namespace Mechanism
