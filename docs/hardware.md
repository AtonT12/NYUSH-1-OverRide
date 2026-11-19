# Hardware sign-off

Everything in this file is **unverified** until a date is written in the
"checked" column. `src/robot-config.cpp` encodes the assumptions below; if a
row fails on the bench, fix that row's reverse flag and nothing else.

Test procedure: mark the robot's front (claw side), run in `RunMode::Bench`
(60 RPM cap) — on blocks first, then on the floor. Never sign a row from a
full-speed run, and change one sign at a time.

## 1. Port map and direction

Positive command is documented per column. "Predicted" comes from existing
hardware notes, not from a measurement.

| # | Logical motor | Port | Cartridge | Reverse flag | Positive command means | Predicted | Checked | Date |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Drive FL A | 1 | 6:1 | yes | wheel drives robot forward | forward | ☐ | |
| 2 | Drive FL B | 2 | 6:1 | yes | wheel drives robot forward | forward | ☐ | |
| 3 | Drive FR A | 4 | 6:1 | no | wheel drives robot forward | forward | ☐ | |
| 4 | Drive FR B | 14 | 6:1 | no | wheel drives robot forward | forward | ☐ | |
| 5 | Drive RL A | 7 | 6:1 | yes | wheel drives robot forward | forward | ☐ | |
| 6 | Drive RL B | 8 | 6:1 | yes | wheel drives robot forward | forward | ☐ | |
| 7 | Drive RR A | 3 | 6:1 | no | wheel drives robot forward | forward | ☐ | |
| 8 | Drive RR B | 19 | 6:1 | no | wheel drives robot forward | forward | ☐ | |
| 9 | Primary lift A | 9 | 18:1 | no | lift raises | raise | ☐ | |
| 10 | Primary lift B | 10 | 18:1 | yes | lift raises | raise | ☐ | |
| 11 | Toggle | 13 | 18:1 | no | toggle raises | raise | ☐ | |
| 12 | Secondary lift | 5 | 18:1 | no | secondary lift raises | raise | ☐ | |
| 13 | Claw | 6 | 18:1 | no | claw closes | close | ☐ | |

**Unresolved:** `NYU-Vex-2026` drives a second coarse-lift motor on **port 15**.
If that motor exists on this robot the count is 14, not 13, and
`secondaryLift` must become a `MotorPair` with port 15 reversed. Confirm before
the first powered test.

**Cartridge check:** the reference uses red (36:1, 100 RPM) on both lift axes,
this project configures 18:1. A 20 RPM command on a 36:1 motor is 20% duty, not
10%. Verify the physical cartridges before trusting any RPM number.

## 2. Mixer sign-off

Run each row at 60 RPM with the robot on blocks, then repeat on the floor with
the robot's "front" marked (claw side).

| Input | Expected physical result | Pass | Date |
| --- | --- | --- | --- |
| Left stick forward | all four wheels drive forward, robot translates toward the claw | ☐ | |
| Left stick right | FL/RR one way, FR/RL the other, robot strafes right without yaw | ☐ | |
| Right stick right | robot yaws clockwise seen from above | ☐ | |
| Forward + right + yaw | no wheel saturates before the others; robot arcs, not pivots | ☐ | |
| Sticks released | all four wheels stop within one control period | ☐ | |

If a row fails, change **one** of: the stick sign in `src/main.cpp`, the mixer
sign in `src/control/kinematics.cpp`, or the motor reverse flag in
`src/robot-config.cpp`. Changing two cancels out and looks correct on blocks.

## 3. Mechanism limits

All values are placeholders copied from the reference build. They must be
measured before closed-loop moves are enabled.

| Axis | Soft min | Soft max | Tolerance | Approach band | Source | Measured | Date |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Primary lift | 0° | 202° | 2° | 15° | `NYU-Vex-2026` `main.rs:269` | ☐ | |
| Secondary lift | unknown | unknown | 2° | 10° | none | ☐ | |
| Claw | open | closed | 2° | 10° | none | ☐ | |
| Toggle | down | up | 2° | 10° | none | ☐ | |

Homing: drive into the mechanical stop, stop when current ≥ 2.5 A **and**
position change ≤ 2° for 10 consecutive ticks, then zero the encoder. A timeout
of 8 s aborts the attempt.

## 4. Unknowns the software is waiting on

1. Driven wheel diameter including compression, and whether the four modules sit
   at exactly 45°.
2. External gear teeth and total ratio per wheel group.
3. Lift reduction ratio, real travel in degrees or millimetres, and where the
   mechanical hard stops are.
4. Holding torque needed at the lift with a full game object (decides whether
   40% torque limit is enough, and whether a spring or counterweight is needed).
5. Claw travel and grip force.
6. Whether the toggle is a ratchet (one-way) or a reversible linkage.
7. Whether an IMU is installed, and on which port (reference uses 18).
8. Battery model, count and main lead gauge.

## 6. Test modes

1. **Direction and mixer rows** — set `kBootRunMode` in `include/constants.h` to
   `RobotConstants::RunMode::Bench` and rebuild. The drivetrain cap drops to
   `kBenchWheelCapRpm` (60 RPM); mechanisms are unaffected, so hold the lift and
   claw inputs briefly and watch one axis at a time.
2. **Competition** — `kBootRunMode` now ships as `RunMode::Field`. If a robot is
   handed to a driver before sections 1 and 2 are signed, set it back to
   `RunMode::Bench`; `RunMode::Locked` keeps output at zero while still showing
   stick values.

## 7. Torque limits

Defaults are `kDriveTorquePercent = 60`, `kMechanismTorquePercent = 40`. If the
lift sags while holding a load, raise the mechanism limit to 60 and re-check
current draw; if the drivetrain browns out on a hard direction change, lower the
drive limit to 40.
