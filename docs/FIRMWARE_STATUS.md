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
It does not identify the exact MCU, motor-driver interface, sensor pin numbers,
remote receiver/API, or C++ library names. Therefore this repository deliberately
does not invent GPIO assignments or claim compatibility with an unverified
Arduino/AVR board.

A board-specific adapter still needs to:
1. read the three IR sensors and normalize their polarity to true = black;
2. read the remote key and map it to robot_b::RemoteKey;
3. call Controller::update() with a millisecond clock;
4. translate signed left/right motor commands to the actual Huna motor-driver API;
5. confirm whether the guide's black-line None signal corresponds to the
   sensor module's electrical HIGH or LOW.

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
