# NYUSH-1 — VEX U Override Small Robot

VEX V5 C++ competition program for the 2026–2027 VEX U Override Small Robot.

> **Current status:** architecture and safety scaffold. All 13 motor bindings are
> intentionally empty because no Smart Port map or per-motor direction data has
> been confirmed. The robot will not drive or move any mechanism in this state.
> No autonomous routine or X-drive mixing formula is implemented yet.

## Robot configuration

| Item | Confirmed configuration |
| --- | --- |
| Competition | 2026–2027 VEX U Override |
| Robot category | Small Robot |
| Maximum starting size | 15 × 15 × 15 in |
| Brain | VEX V5 Brain |
| Controller | VEX V5 Controller |
| Drivetrain | 8-motor Omni X-Drive |
| Drivetrain layout | Four Omni wheels; two motors per wheel through a gear transmission |
| Toggle | 1 V5 Smart Motor |
| Primary vertical lift | 2 V5 Smart Motors, one lifting assembly |
| Secondary lift | 1 V5 Smart Motor |
| Claw / Pin gripper | 1 V5 Smart Motor |
| Total motors | 13 |

This is a conventional four-wheel Omni X-drive, **not** a Mecanum drivetrain.
The drivetrain motor pairs must not be assumed to share the same mechanical
direction. Each motor needs its own reverse setting after physical verification.

The starting-size limit is a mechanical compliance requirement. Software does
not enforce it, and satisfying it does not waive any other VEX U expansion or
robot rule.

## Development environment

- IDE: Visual Studio Code
- Extension: official VEX Robotics VEXcode extension `0.8.2026020401`
- SDK: `V5_20240802_15_00_00`
- Language: C++11 (`-std=gnu++11` from the VEX makefile)
- Project type: VEXcode V5 competition template with the VEX makefile
- Program slot: 1

The project does not use PROS, a custom PID controller, an IMU, odometry, path
following or a third-party library.

### macOS Apple Silicon note

The installed VEX toolchain binaries (`clang`, `make`, `arm-none-eabi-*`) are
x86_64. On this arm64 Mac, Rosetta is not installed, so the VEX toolchain cannot
execute. The VEX extension's build path also contains an explicit Rosetta
prerequisite check. A full build and link require either:

- installing Rosetta for this toolchain, or
- using a macOS/Windows build host with a working VEX toolchain.

Do not claim a successful full build until that environment issue is resolved.

### Verification status (2026-11-13)

- All nine C++ translation units and every project header passed a syntax check
  against the installed VEX SDK headers using the host `clang++` with the VEX
  target flags.
- Each source also compiled to an ARM ELF object in a temporary directory.
- The project's own VEX makefile was attempted. It stopped at the first source
  with `Bad CPU type in executable` because the bundled x86_64 `clang` cannot
  run on this arm64 host without Rosetta.
- Therefore the official link, `.bin` generation, download and hardware test
  have **not** been performed. The temporary ARM objects are a partial
  verification only.

## Directory structure

```text
NYUSH-1/
├── include/
│   ├── vex.h                 # Unchanged VEX template wrapper header
│   ├── constants.h           # Loop period, deadband, output limits
│   ├── robot-config.h        # Brain, Controller and 13 motor binding slots
│   ├── motor-control.h       # Null-safe motor helpers
│   ├── drivetrain.h          # Wheel-group hardware interface
│   ├── toggle.h              # Toggle mechanism interface
│   ├── vertical-lift.h       # Two-motor primary lift interface
│   ├── secondary-lift.h      # Secondary lift interface
│   ├── claw.h                # Pin gripper interface
│   └── autonomous.h          # Autonomous entry point
├── src/
│   ├── main.cpp              # Competition callbacks and teleop loop
│   ├── robot-config.cpp      # Central device bindings and readiness checks
│   ├── motor-control.cpp     # Deadband, limiting, spin/stop helpers
│   ├── drivetrain.cpp        # Low-level four-wheel-group output layer
│   ├── toggle.cpp            # Toggle implementation
│   ├── vertical-lift.cpp     # Coordinated two-motor lift implementation
│   ├── secondary-lift.cpp    # Secondary lift implementation
│   ├── claw.cpp              # Claw implementation
│   └── autonomous.cpp        # Autonomous strategy entry point
├── vex/                      # Unchanged VEX build rules
├── .vscode/                  # VEX project/IntelliSense settings
├── makefile                  # Unchanged VEX makefile
└── README.md
```

The VEX makefile automatically includes every `.cpp` directly under `src/`.
No makefile change is needed when these files are added.

## Module responsibilities

| Module | Responsibility |
| --- | --- |
| `robot-config` | Single place for Brain, Controller and all future motor bindings; readiness checks; global stop |
| `motor-control` | Shared deadband, magnitude limit, signed percent command and null-safe stop |
| `drivetrain` | Four logical wheel groups, encoder reset and low-level wheel power; intentionally no X-drive mixing yet |
| `toggle` | Field Toggle motor power and stop |
| `vertical-lift` | Commands both primary-lift motors together as one subsystem |
| `secondary-lift` | Articulated secondary-lift motor power and stop |
| `claw` | Pin gripper motor power and stop |
| `autonomous` | Strategy entry point for a future verified routine |

`robot-config` does not depend on any mechanism module. Mechanism and drivetrain
modules depend on `robot-config`, and `main.cpp` depends on them. There are no
cyclic includes.

## Current competition behavior

### Pre-autonomous

- Stops every bound motor.
- Resets drivetrain motor encoders only if all eight drivetrain motors are
  bound. The current bindings are empty, so this is a no-op.
- Shows project, hardware-binding and autonomous status on the Brain screen.
- Does not perform fictional IMU, gyro or sensor calibration.

### Autonomous

- Stops every bound motor on entry.
- Calls `Autonomous::run()`.
- Stops every bound motor on exit.
- No route is implemented. Real route work requires verified drivetrain geometry,
  start positions and mechanism travel limits.

### Driver control

- Stops every bound motor on entry.
- Reads the forward axis (`Controller.Axis3`) and turn axis (`Controller.Axis1`)
  every 20 ms through the configured deadband.
- Displays the received values on the Brain for controller-path debugging.
- Does **not** command the drivetrain because the X-drive geometry and motor
  directions are unverified.
- Does not yet map controller buttons to mechanisms.

### Stopping behavior

All module stop paths use `vex::brakeType::brake` as the first-version default.
Mechanism-specific `hold`/`coast` behavior remains to be selected after the
mechanical team characterizes each axis.

## Hardware binding policy

`src/robot-config.cpp` currently binds all motors as follows:

```cpp
RobotHardware::driveFrontLeft = {nullptr, nullptr};
// ... three more drivetrain pairs, primary lift pair, three single motors
```

`nullptr` means **unassigned hardware**, not a valid port or a simulated motor.
Do not replace it with a guessed `PORTx` value.

When the port map is confirmed:

1. Declare each motor once in `src/robot-config.cpp` with its confirmed port,
   cartridge and independent reverse setting.
2. Point the corresponding `RobotHardware` binding at the declared motor.
3. Keep all device declarations centralized; do not declare motors in module
   source files.
4. Verify each drivetrain wheel direction at low speed before adding any mixing.
5. Verify each mechanism's positive direction and safe travel before enabling
   operator controls.

The modules require a complete pair for two-motor subsystems. If only one lift
or drivetrain motor is bound, that subsystem reports not ready and refuses to
command either motor, while `stopAllMotors()` can still stop any bound motor.

## Hardware information still required

Fill this table before connecting any power or running motion tests.

| Logical device | Binding field | Smart Port | Cartridge | Reverse | Notes |
| --- | --- | --- | --- | --- | --- |
| Drive FL motor A | `driveFrontLeft.motorA` | TBD | TBD | TBD | 2-motor gear-driven wheel |
| Drive FL motor B | `driveFrontLeft.motorB` | TBD | TBD | TBD | Verify independently |
| Drive FR motor A | `driveFrontRight.motorA` | TBD | TBD | TBD | 2-motor gear-driven wheel |
| Drive FR motor B | `driveFrontRight.motorB` | TBD | TBD | TBD | Verify independently |
| Drive RL motor A | `driveRearLeft.motorA` | TBD | TBD | TBD | 2-motor gear-driven wheel |
| Drive RL motor B | `driveRearLeft.motorB` | TBD | TBD | TBD | Verify independently |
| Drive RR motor A | `driveRearRight.motorA` | TBD | TBD | TBD | 2-motor gear-driven wheel |
| Drive RR motor B | `driveRearRight.motorB` | TBD | TBD | TBD | Verify independently |
| Toggle | `toggleMotor` | TBD | TBD | TBD | |
| Primary lift A | `primaryLift.motorA` | TBD | TBD | TBD | Coupled to lift B |
| Primary lift B | `primaryLift.motorB` | TBD | TBD | TBD | Verify independently |
| Secondary lift | `secondaryLiftMotor` | TBD | TBD | TBD | Articulated lift |
| Claw | `clawMotor` | TBD | TBD | TBD | Pin gripper |

Also required:

- Gear tooth counts and gear ratio for each wheel group.
- Wheel diameter and exact wheel mounting orientation.
- Allowed lift travel/angle and safe speed limits.
- Mechanism stop/hold requirements.
- Operator button mapping.

## Build and download

### From VS Code

1. Open this folder in VS Code with the VEX extension installed.
2. Open the VEX sidebar and use **Build**.
3. Review all compiler and linker output before downloading.
4. Download only after the hardware configuration is reviewed and the Brain is
   physically safe to test.
5. Download to program slot 1 as configured in `.vscode/vex_project_settings.json`.

### From a terminal (same VEX makefile)

```bash
TOOLCHAIN="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_osxarm64"
SDK_HOME="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00"

PATH="$TOOLCHAIN/gcc/bin:$TOOLCHAIN/tools/bin:$TOOLCHAIN/clang/bin:$PATH" \
  make T="$SDK_HOME" P=NYUSH-1
```

This command is the project's own VEX build flow, not a generic host build. On
the current arm64 Mac it fails at the first compile until Rosetta is installed.
The generated `build/` directory is ignored by `.gitignore`.

Do not download or run the program on a physical Brain without explicit user
authorization and a safe mechanical test setup.

## Next development steps

1. Record the confirmed 13-port map, cartridge, and independent reverse flag for
   every motor.
2. Bind the motors in `robot-config.cpp` and verify readiness/stop behavior.
3. Verify each drivetrain wheel group at low speed, then implement one mixing
   axis at a time: forward/reverse, strafe, rotation, then diagonal.
4. Add operator bindings for Toggle, Primary Lift, Secondary Lift and Claw only
   after safe direction and travel limits are known.
5. Add encoder-based movement helpers for mechanisms with verified limits.
6. Develop the autonomous routine only after chassis motion is repeatable.
