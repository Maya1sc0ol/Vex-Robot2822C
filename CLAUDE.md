# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a VEX Robotics V5 project using [PROS](https://pros.cs.purdue.edu/) (Purdue Robotics Operating System), a open-source C/C++ development environment for VEX V5 brains. The target device is the **VEX EDR V5** (ARM Cortex-A9, `arm-none-eabi` toolchain).

## Build Commands

Requires the PROS CLI and `arm-none-eabi` cross-compiler toolchain installed.

```sh
make          # Incremental build (default: quick target, builds hot.package.bin)
make all      # Clean then full build
make clean    # Remove bin/ and .d/ directories
```

The PROS CLI is also available for uploading and managing the project:
```sh
pros build             # Same as make quick
pros upload            # Upload to connected V5 brain
pros build-upload      # Build and upload in one step
pros terminal          # Open serial terminal to V5
```

## Architecture

### Hot/Cold Linking

The project uses PROS hot/cold linking (`USE_PACKAGE=1` in `Makefile`):
- **Cold package** (`bin/cold.package.bin`): Contains PROS kernel + libraries. Built once and cached on the V5 brain.
- **Hot package** (`bin/hot.package.bin`): Contains user code only. Uploaded on every change (much faster).

The linker scripts `firmware/v5.ld`, `firmware/v5-hot.ld`, and `firmware/v5-common.ld` control this split.

### Competition Lifecycle Functions

All robot logic lives in `src/main.cpp` through five required entry points called by the PROS scheduler:

| Function | When it runs |
|---|---|
| `initialize()` | On program start, blocks all other modes |
| `disabled()` | When FMS disables the robot |
| `competition_initialize()` | After `initialize()`, before autonomous, when connected to FMS |
| `autonomous()` | During the autonomous period |
| `opcontrol()` | During driver control |

### Key Namespaces and APIs

- `pros::Motor` / `pros::MotorGroup` — motor control (ports are 1-indexed; negative port = reversed)
- `pros::Controller` — reads joystick axes and buttons
- `pros::lcd` — LLEMU (Legacy LCD Emulator) for the V5 brain screen
- `pros::Task`, `pros::Mutex`, `pros::delay()` — RTOS primitives
- `pros::Imu`, `pros::Rotation`, `pros::Distance`, `pros::Optical` — sensor classes
- `pros::ADIDigitalOut`, `pros::ADIEncoder` etc. — ADI (3-wire) port devices

`PROS_USE_SIMPLE_NAMES` is defined in `include/main.h`, enabling shorthand enum names (e.g., `CONTROLLER_MASTER` instead of `E_CONTROLLER_MASTER`).

### Adding Code

- Add new `.cpp` files anywhere under `src/` — the Makefile recursively discovers them.
- Add headers to `include/` — both `include/` and the file's own directory are on the include path.
- The `opcontrol()` loop runs every 20 ms (`pros::delay(20)`); keep tasks in that loop brief or offload to separate PROS tasks.

### Standard Patterns

**Motor group (drivetrain):**
```cpp
pros::MotorGroup left_mg({1, -2, 3});   // negative port = reversed
pros::MotorGroup right_mg({-4, 5, -6});
```

**Arcade drive:**
```cpp
int dir  = master.get_analog(ANALOG_LEFT_Y);
int turn = master.get_analog(ANALOG_RIGHT_X);
left_mg.move(dir - turn);
right_mg.move(dir + turn);
```

**Separate PROS task:**
```cpp
pros::Task my_task([]() {
    while (true) {
        // sensor update, etc.
        pros::delay(10);
    }
});
```
