# Firmware logic status

## What is implemented

- Portable C++17 control logic for the two Bảng B tasks described in the supplied
  Hướng dẫn lập trình Robot Bảng B slides:
  - line-following from three logical IR inputs;
  - left/right corner handling;
  - first all-black crossing: drive straight for a configurable duration;
  - second all-black crossing: stop and switch to remote mode;
  - remote forward, backward, left, right, and key-off stop.
- Host-side tests that can be run without connecting the robot.

## Important hardware boundary

This is CONTROL LOGIC, not a board-ready firmware image. The slide deck names
E-Robot V1.4 and a Huna mainboard, and describes MAKE/DOWNLOAD in that software.
The user supplied a Huna API header named `functions_codes.h` and a sample program. A board-oriented example is now in `firmware/huna_main.c`. The exact MCU, physical sensor placement, and real sensor polarity still have not been independently verified; no GPIO assignments are needed in this API-level example.

The Huna example currently assumes:
1. `ir(1, 0, NONE) == TRUE` means the left sensor detects black, likewise sensors 2 and 3 for center/right;
2. `rc(UP_KEY/DOWN_KEY/LEFT_KEY/RIGHT_KEY) == TRUE` reports the corresponding remote key;
3. `LMotor_1()` and `RMotor_1()` accept signed speed values as described by the supplied header;
4. motor speed 5 and crossing delay 250 ms are starting values, not measured settings.

Verify sensor placement/polarity and turning direction on the real robot before track use. The separate portable C++ core still uses a board adapter for integration with `robot_logic.hpp`.

The guide says positive motor speed means forward, negative means reverse, and
zero means stop. The core uses this convention. The physical left/right wiring
must be checked on the actual robot.

## Build and test on a development computer

Requirements: CMake 3.16+ and a C++17 compiler.

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

These are host-side logic tests only; they do not test motors, sensors, battery
capacity, the E-Robot compiler, or downloading to the mainboard.

## Tune before running

Config defaults are conservative starting values, not measured hardware
settings. Adjust forward_speed, turn_speed, and first_cross_drive_ms on a
safe test track. The all-sensors-black pattern is used by the supplied guide as
the crossing condition; verify the actual sensor placement and polarity first.
