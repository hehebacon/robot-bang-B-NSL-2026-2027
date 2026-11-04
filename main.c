/*
 * BANG B - HUNA E-ROBOT V1.4
 * Compatibility-focused firmware entry point.
 *
 * This version intentionally avoids float arithmetic, stdio, large helper
 * structures, and software ramp calculations. It uses only the Huna API
 * already present in the supplied working example.
 *
 * Sensor convention copied from the supplied example:
 *   ir(n, 0, NONE) == TRUE means sensor n detects black.
 *
 * IMPORTANT:
 * - Verify sensor order and motor direction on the real robot.
 * - The first all-black crossing advances to stage 2.
 * - The second all-black crossing stops and enters remote control.
 * - This source simplification cannot by itself prove/fix a USB/bootloader
 *   transfer stall at 50%; it removes source-level compatibility variables.
 */
#include "functions_codes.h"

#define DRIVE_SPEED 5
#define TURN_SPEED  5
#define CROSSING_MS 250

static void motors_stop(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char sensor_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char crossing_detected(void)
{
    return (sensor_black(1) &&
            sensor_black(2) &&
            sensor_black(3));
}

/* Three-sensor line tracking. No floating-point calculations or timers. */
static void line_follow(void)
{
    unsigned char left;
    unsigned char center;
    unsigned char right;

    left = sensor_black(1);
    center = sensor_black(2);
    right = sensor_black(3);

    if (left && center && right) {
        motors_stop();
    } else if (left && center) {
        LMotor_1(-TURN_SPEED);
        RMotor_1(TURN_SPEED);
    } else if (center && right) {
        LMotor_1(TURN_SPEED);
        RMotor_1(-TURN_SPEED);
    } else if (left) {
        LMotor_1(-TURN_SPEED);
        RMotor_1(TURN_SPEED);
    } else if (right) {
        LMotor_1(TURN_SPEED);
        RMotor_1(-TURN_SPEED);
    } else if (center) {
        LMotor_1(DRIVE_SPEED);
        RMotor_1(DRIVE_SPEED);
    } else {
        /* Lost line: stop rather than keep the previous motor command. */
        motors_stop();
    }
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            LMotor_1(DRIVE_SPEED);
            RMotor_1(DRIVE_SPEED);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-DRIVE_SPEED);
            RMotor_1(-DRIVE_SPEED);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(-TURN_SPEED);
            RMotor_1(TURN_SPEED);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(TURN_SPEED);
            RMotor_1(-TURN_SPEED);
        } else {
            /* No key (including OFF) means stop. */
            motors_stop();
        }
    }
}

int main(void)
{
    mcu_initialization();
    motors_stop();

    /* Stage 1: follow the line until the first crossing. */
    for (;;) {
        if (crossing_detected()) {
            LMotor_1(DRIVE_SPEED);
            RMotor_1(DRIVE_SPEED);
            delay(0, 0, 0, CROSSING_MS);
            motors_stop();
            break;
        }
        line_follow();
    }

    /* Stage 2: follow the line until the second crossing. */
    for (;;) {
        if (crossing_detected()) {
            motors_stop();
            delay(0, 0, 0, CROSSING_MS);
            motors_stop();
            break;
        }
        line_follow();
    }

    /* Part 2: remote-control mode. */
    remote_control();

    return 0;
}
