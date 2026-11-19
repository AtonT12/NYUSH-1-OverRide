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
| 9 | Primary lift A | 9 | 36:1 | no | lift raises | raise | ☐ | |
| 10 | Primary lift B | 10 | 36:1 | yes | lift raises | raise | ☐ | |
| 11 | Toggle | 13 | 18:1 | no | toggle raises | raise | ☐ | |
| 12 | Secondary lift | 5 | 36:1 | no | secondary lift raises | raise | ☐ | |
| 13 | Claw | 6 | 18:1 | no | claw closes | close | ☐ | |

**Unresolved:** `NYU-Vex-2026` drives a second coarse-lift motor on **port 15**.
If that motor exists on this robot the count is 14, not 13, and
`secondaryLift` must become a two-motor pair with port 15 reversed. Confirm
before the first powered test.

**Cartridge check:** the two lift axes are configured 36:1 to match the
reference's red cartridges; claw and toggle stay 18:1 green. The gearset
argument only scales RPM into duty, so a wrong value does not make a motor
misbehave visibly — it just runs at half or double the commanded speed.

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
| Fine lift (9, 10) | -20° | none | 2° | — | `NYU-Vex-2026` `main.rs:335` | ☐ | |
| Coarse lift (5) | none | min + 202° | 2° | 15° | `NYU-Vex-2026` `main.rs:269` | ☐ | |
| Claw | open | closed | 2° | 10° | none | ☐ | |
| Toggle | down | up | 2° | 10° | none | ☐ | |

The coarse limit is implemented in `src/coarse-lift.cpp`, ported verbatim: the
lowest negative angle the motor has reported becomes the bottom of travel, and
the last 15° before the cap hand over to a 12 RPM position target. **Power the
brain on with the coarse lift down**, otherwise the bottom reference is wherever
it happened to be and the cap sits past the real stop.

The fine lift has no upper soft limit in the reference either; it is stopped by
its mechanical stop at 100 RPM. Watch the first few presses.

Planned, not implemented: homing by current (stop when current ≥ 2.5 A **and**
position change ≤ 2° for 10 consecutive ticks, then zero the encoder; abort after
8 s).

## 4. Unknowns the software is waiting on

1. Driven wheel diameter including compression, and whether the four modules sit
   at exactly 45°.
2. External gear teeth and total ratio per wheel group.
3. Lift reduction ratio, real travel in degrees or millimetres, and where the
   mechanical hard stops are.
4. Holding torque needed at the lift with a full game object (decides whether
   the 36:1 motors hold on their own or a spring/counterweight is needed; the
   motors run unlimited, so a sag here is a mechanical answer, not a software
   one).
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

There are none. `applyMotorConfiguration()` only sets stopping modes; no
`setMaxTorque` call exists. The reference drives every motor at its own current
limit, and a partial limit is felt as "the whole robot is weak" long before it
protects anything.

If the drivetrain browns out on a hard direction change, or the lift overheats
while holding a load, the knob to reintroduce is `setMaxTorque(percent,
percentUnits::pct)` on the specific motors — not on all thirteen.
