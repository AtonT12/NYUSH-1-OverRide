# NYUSH-1 — VEX U Override Small Robot

VEX V5 C++ competition program for the 2026–2027 VEX U Override Small Robot.

> **Current status:** all 13 motors are bound to their confirmed Smart Ports with
> per-motor direction normalization. The competition program still does not
> command the drivetrain or mechanisms in teleop, because the full X-drive mixing
> convention and operator button map are not yet validated. No autonomous routine
> is implemented.

## Robot configuration

| Item | Confirmed configuration |
| --- | --- |
| Competition | 2026–2027 VEX U Override |
| Robot category | Small Robot |
| Maximum starting size | 15 × 15 × 15 in |
| Brain | VEX V5 Brain |
| Controller | VEX V5 Controller |
| Robot orientation | Claw side = front; Toggle side = rear |
| Drivetrain | 8-motor Omni X-Drive |
| Drivetrain layout | Four Omni wheels; two motors per wheel through a gear transmission |
| Toggle | 1 V5 Smart Motor |
| Primary vertical lift | 2 V5 Smart Motors, one lifting assembly |
| Secondary lift | 1 V5 Smart Motor |
| Claw / Pin gripper | 1 V5 Smart Motor |
| Total motors | 13 |

This is a conventional four-wheel Omni X-drive, **not** a Mecanum drivetrain.
Every drive motor has its own confirmed reverse setting. The current code
normalizes each wheel group so a positive Drivetrain command drives that wheel
forward; this is a hardware-level convention, not yet the final chassis mixing
formula.

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
x86_64. This arm64 Mac now has Rosetta installed, and the toolchain executes.
On another Apple Silicon build host without Rosetta, install it before using the
VEX build command.

### Verification status (2026-11-13)

- A clean official VEX build was run with the bundled VEX makefile, compiler and
  linker. All nine translation units compiled, linked and produced
  `build/NYUSH-1.bin`.
- All project headers also passed independent syntax checks against the VEX SDK
  headers. The earlier host-`clang++` object compilation passed as a supplementary
  check.
- No program has been downloaded to a physical Brain, and no hardware motion test
  has been performed.

## Directory structure

```text
NYUSH-1/
├── include/
│   ├── vex.h                 # Unchanged VEX template wrapper header
│   ├── constants.h           # Loop period, deadband, output limits
│   ├── robot-config.h        # Brain, Controller and 13 motor bindings
│   ├── motor-control.h       # Null-safe motor helpers
│   ├── drivetrain.h          # Wheel-group hardware interface
│   ├── toggle.h              # Toggle mechanism interface
│   ├── vertical-lift.h       # Two-motor primary lift interface
│   ├── secondary-lift.h      # Secondary lift interface
│   ├── claw.h                # Pin gripper interface
│   └── autonomous.h          # Autonomous entry point
├── src/
│   ├── main.cpp              # Competition callbacks and teleop loop
│   ├── robot-config.cpp      # Confirmed ports, directions and readiness checks
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
| `robot-config` | Single place for Brain, Controller and all 13 motor bindings; direction normalization; readiness checks; global stop |
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
- Resets all eight drivetrain motor encoders to zero after verifying the
  drivetrain binding is complete. This is a position reset only; it does not move
  the robot and assumes the robot is at rest.
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
- Does **not** command the drivetrain because the complete X-drive wheel-axis
  mixing convention is not yet validated. Per-motor directions are confirmed and
  normalized.
- Does not yet map controller buttons to mechanisms.

### Stopping behavior

All module stop paths use `vex::brakeType::brake` as the first-version default.
Mechanism-specific `hold`/`coast` behavior remains to be selected after the
mechanical team characterizes each axis.

## Confirmed hardware bindings

All 13 motor objects are declared once in `src/robot-config.cpp` and exposed
through the centralized `RobotHardware` bindings. The reverse flag is applied to
the motor object, not hidden inside a mechanism module.

| Logical motor | Binding | Port | Native positive | Reverse flag | Logical positive |
| --- | --- | --- | --- | --- | --- |
| Drive FL A | `driveFrontLeft.motorA` | 1 | backward | Yes | wheel forward |
| Drive FL B | `driveFrontLeft.motorB` | 2 | backward | Yes | wheel forward |
| Drive FR A | `driveFrontRight.motorA` | 4 | forward | No | wheel forward |
| Drive FR B | `driveFrontRight.motorB` | 14 | forward | No | wheel forward |
| Drive RL A | `driveRearLeft.motorA` | 7 | backward | Yes | wheel forward |
| Drive RL B | `driveRearLeft.motorB` | 8 | backward | Yes | wheel forward |
| Drive RR A | `driveRearRight.motorA` | 3 | forward | No | wheel forward |
| Drive RR B | `driveRearRight.motorB` | 19 | forward | No | wheel forward |
| Toggle | `toggleMotor` | 13 | upward | No | upward |
| Primary lift A | `primaryLift.motorA` | 9 | raise | No | raise |
| Primary lift B | `primaryLift.motorB` | 10 | lower | Yes | raise |
| Secondary lift | `secondaryLiftMotor` | 5 | raise | No | raise |
| Claw | `clawMotor` | 6 | close | No | close |

Direction normalization notes:

- FL and RL are reverse-normalized because their native positive direction is
  backward. FR and RR are not reversed. Positive Drivetrain power therefore
  means "wheel forward" for every group after normalization.
- Primary Lift B is reverse-normalized because its native positive direction
  lowers the assembly. Positive VerticalLift power therefore raises both sides
  together.
- Toggle, Secondary Lift and Claw already have native positive directions that
  match their module conventions.
- Motor cartridges are still not encoded. The motors use the two-argument
  constructor (port + reverse), and all current commands are percent-based; no
  RPM, wheel-distance or gear-ratio conversion is performed.
- The modules still require complete two-motor pairs. If a binding is later
  removed or only one side is available, the subsystem refuses to move while
  `stopAllMotors()` can still stop the remaining motor.

This normalization is not the final X-drive mixing formula. Do not connect wheel
commands directly to the operator joystick until the complete wheel-axis sign
convention has been tested.

## Hardware information still required

- Cartridge type for all 13 motors.
- Gear tooth counts and gear ratio for each wheel group.
- Wheel diameter and exact wheel mounting/axis convention for X-drive mixing.
- Allowed lift travel/angle and safe speed limits.
- Mechanism stop/hold requirements.
- Operator button mapping.
- Autonomous starting positions and strategy.

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

This command is the project's own VEX build flow, not a generic host build. The
current arm64 Mac has Rosetta installed and completed this build successfully.
The generated `build/` directory is ignored by `.gitignore`.

Do not download or run the program on a physical Brain without explicit user
authorization and a safe mechanical test setup.

## Next development steps

1. Download to the Brain (only when explicitly authorized and physically safe),
   verify it boots to pre-auton, reports `HW bindings: ready`, and shows no
   motor movement while the field is disabled.
2. Bench-test each drive wheel group at very low power, one group at a time, and
   confirm the normalized positive direction.
3. Implement one chassis mixing axis at a time after direction testing:
   forward/reverse, strafe, rotation, then diagonal.
4. Add operator bindings for Toggle, Primary Lift, Secondary Lift and Claw only
   after safe direction and travel limits are known.
5. Add encoder-based movement helpers for mechanisms with verified limits.
6. Develop the autonomous routine only after chassis motion is repeatable.
