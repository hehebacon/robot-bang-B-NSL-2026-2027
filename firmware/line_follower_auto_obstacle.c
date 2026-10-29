/*
 * BANG B - HUNA E-ROBOT V1.4
 * Fresh standalone firmware: line following + fixed obstacle-pushing bar.
 *
 * This file is intentionally separate from the older firmware files.
 *
 * Initial assumptions to verify on the real robot:
 * - IR1 = left, IR2 = center, IR3 = right.
 * - ir(n, 0, NONE) == TRUE means sensor n sees the black line.
 * - Positive motor speed moves the robot forward.
 * - LMotor_1(0), RMotor_1(5) curves left; LMotor_1(5), RMotor_1(0) curves right.
 * - A crossing is detected when all three sensors see black.
 *
 * The pushing bar is fixed mechanically, so no servo command is used for it.
 * Speeds and timing are starting values, not physically calibrated values.
 */
#include "functions_codes.h"

#define BASE_SPEED                 5
#define CROSSING_CLEAR_SPEED       4
#define FIRST_CROSSING_PASS_MS   250
#define CROSSING_CLEAR_STEP_MS    10
#define CROSSING_CLEAR_MAX_STEPS 200
#define LOST_LINE_SEARCH_STEPS    25
#define LOOP_DELAY_MS               2

#define STAGE_AUTO_1 0
#define STAGE_AUTO_2 1

static int last_turn = 0; /* -1 = left, +1 = right, 0 = unknown */

static void motors_stop(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char sees_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char crossing_detected(void)
{
    return (sees_black(1) && sees_black(2) && sees_black(3));
}

static void drive_forward(unsigned char speed)
{
    LMotor_1(speed);
    RMotor_1(speed);
}

/*
 * Huna-style differential steering: keep both motors forward and slow one
 * side by stopping it. This avoids assuming reverse commands are needed.
 */
static void steer_left(void)
{
    LMotor_1(0);
    RMotor_1(BASE_SPEED);
    last_turn = -1;
}

static void steer_right(void)
{
    LMotor_1(BASE_SPEED);
    RMotor_1(0);
    last_turn = 1;
}

static void follow_black_line(void)
{
    unsigned char left = sees_black(1);
    unsigned char center = sees_black(2);
    unsigned char right = sees_black(3);

    if (left && center && right) {
        /* The stage controller handles crossings. */
        motors_stop();
    } else if (left && center) {
        steer_left();
    } else if (center && right) {
        steer_right();
    } else if (left) {
        steer_left();
    } else if (right) {
        steer_right();
    } else if (center) {
        drive_forward(BASE_SPEED);
    } else {
        /*
         * Briefly search in the last known direction. If the line is still
         * missing after a bounded search, stop rather than drive blindly.
         */
        unsigned int step;
        for (step = 0; step < LOST_LINE_SEARCH_STEPS; ++step) {
            if (sees_black(1) || sees_black(2) || sees_black(3)) {
                return;
            }

            if (last_turn < 0) {
                steer_left();
            } else if (last_turn > 0) {
                steer_right();
            } else {
                motors_stop();
                return;
            }
            delay(0, 0, 0, LOOP_DELAY_MS);
        }
        motors_stop();
    }
}

/* Pass the first crossing and wait for the sensors to leave the black area. */
static unsigned char pass_first_crossing(void)
{
    unsigned int step;

    drive_forward(CROSSING_CLEAR_SPEED);
    delay(0, 0, 0, FIRST_CROSSING_PASS_MS);

    for (step = 0; step < CROSSING_CLEAR_MAX_STEPS; ++step) {
        if (!crossing_detected()) {
            motors_stop();
            delay(0, 0, 0, 50);
            return 1;
        }

        drive_forward(CROSSING_CLEAR_SPEED);
        delay(0, 0, 0, CROSSING_CLEAR_STEP_MS);
    }

    motors_stop();
    return 0; /* fail stopped if the crossing never clears */
}

static void manual_remote_mode(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            drive_forward(BASE_SPEED);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-BASE_SPEED);
            RMotor_1(-BASE_SPEED);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(BASE_SPEED);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(BASE_SPEED);
            RMotor_1(0);
        } else {
            /* No key / OFF_KEY: stop. */
            motors_stop();
        }
        delay(0, 0, 0, LOOP_DELAY_MS);
    }
}

int main(void)
{
    unsigned char stage = STAGE_AUTO_1;

    mcu_initialization();
    motors_stop();

    for (;;) {
        if (crossing_detected()) {
            motors_stop();

            if (stage == STAGE_AUTO_1) {
                if (!pass_first_crossing()) {
                    /* Safe failure: stop permanently if the crossing won't clear. */
                    for (;;) {
                        motors_stop();
                        delay(0, 0, 0, 100);
                    }
                }
                stage = STAGE_AUTO_2;
            } else {
                /* Second crossing ends AUTO and starts remote control. */
                motors_stop();
                delay(0, 0, 0, 250);
                manual_remote_mode();
            }
        } else {
            follow_black_line();
        }

        delay(0, 0, 0, LOOP_DELAY_MS);
    }

    return 0;
}
