/*
 * BANG B - HUNA E-ROBOT V1.4
 * main.c - Huna C firmware (not desktop C++).
 *
 * AUTO: follow the line, ramp speed smoothly, slow for sensor-detected turns,
 * pass two crossings, then enter MANUAL.
 * MANUAL: UP ramps forward speed toward the configured maximum; DOWN acts as
 * a brake command and ramps the target to zero (it does NOT reverse the robot).
 * LEFT/RIGHT steer at the current ramped speed.
 *
 * Hardware assumptions still requiring verification:
 * - IR1=left, IR2=center, IR3=right.
 * - ir(n, 0, NONE)==TRUE means that sensor sees black.
 * - Huna motor API accepts integer speeds in the configured range.
 * - Crossing patterns and timing match the actual course.
 *
 * <bits/stdc++.h> is a GNU C++ header, not valid for this Huna C firmware.
 * Do not add it here; this file uses functions_codes.h.
 */
#include <stdio.h>
#include "functions_codes.h"

#define SPEED_STRAIGHT          8
#define SPEED_MEDIUM            5
#define SPEED_TIGHT              3
#define CROSSING_SPEED           3
#define CROSSING_PASS_MS       180
#define CROSSING_SAMPLE_MS       5
#define CROSSING_STABLE_SAMPLES  3
#define CROSSING_MAX_MS       1250
#define LOST_SEARCH_MS         120
#define LOOP_DELAY_MS            2
#define RAMP_INTERVAL_MS        20
#define SPEED_STEP             0.1f
#define SPEED_MIN              0.0f
#define SPEED_MAX              8.0f

#define STAGE_FIRST              0
#define STAGE_SECOND             1

static int last_turn = 0; /* -1 = left, +1 = right, 0 = unknown */
static float current_speed = 0.0f;
static float target_speed = 0.0f;
static unsigned int ramp_elapsed_ms = 0;

static void stop_drive(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

/* Move current_speed toward target_speed by 0.1 every 20 ms. */
static void update_speed_ramp(void)
{
    ramp_elapsed_ms += LOOP_DELAY_MS;
    if (ramp_elapsed_ms < RAMP_INTERVAL_MS) {
        return;
    }
    ramp_elapsed_ms = 0;

    if (current_speed < target_speed) {
        current_speed += SPEED_STEP;
        if (current_speed > target_speed) current_speed = target_speed;
    } else if (current_speed > target_speed) {
        current_speed -= SPEED_STEP;
        if (current_speed < target_speed) current_speed = target_speed;
    }

    if (current_speed < SPEED_MIN) current_speed = SPEED_MIN;
    if (current_speed > SPEED_MAX) current_speed = SPEED_MAX;
}

/* Huna motor commands appear integer-based; float ramp is rounded down here. */
static int motor_speed(void)
{
    return (int)(current_speed + 0.0001f);
}

static void drive_forward_ramped(void)
{
    int speed = motor_speed();
    LMotor_1(speed);
    RMotor_1(speed);
}

static unsigned char black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char all_black(void)
{
    return (black(1) && black(2) && black(3));
}

static unsigned char crossing_stable(void)
{
    unsigned char count = 0;
    unsigned char i;

    for (i = 0; i < CROSSING_STABLE_SAMPLES; ++i) {
        if (all_black()) ++count;
        delay(0, 0, 0, CROSSING_SAMPLE_MS);
    }
    return (count == CROSSING_STABLE_SAMPLES);
}

/* Set the speed target from the current 3-sensor pattern, then ramp toward it. */
static void follow_line_adaptive(void)
{
    unsigned char left = black(1);
    unsigned char center = black(2);
    unsigned char right = black(3);
    int speed;

    if (left && center && right) {
        target_speed = 0.0f; /* crossing controller handles this pattern */
        update_speed_ramp();
        stop_drive();
        return;
    }

    if (left && center) {
        target_speed = SPEED_MEDIUM;
        last_turn = -1;
    } else if (center && right) {
        target_speed = SPEED_MEDIUM;
        last_turn = 1;
    } else if (left) {
        target_speed = SPEED_TIGHT;
        last_turn = -1;
    } else if (right) {
        target_speed = SPEED_TIGHT;
        last_turn = 1;
    } else if (center) {
        target_speed = SPEED_STRAIGHT;
    } else {
        unsigned int elapsed = 0;
        target_speed = 0.0f;
        update_speed_ramp();
        stop_drive();

        /* Search briefly at low speed in the last known direction. */
        while (elapsed < LOST_SEARCH_MS) {
            if (black(1) || black(2) || black(3)) return;

            speed = motor_speed();
            if (speed > SPEED_TIGHT) speed = SPEED_TIGHT;
            if (last_turn < 0) {
                LMotor_1(0);
                RMotor_1(speed);
            } else if (last_turn > 0) {
                LMotor_1(speed);
                RMotor_1(0);
            } else {
                stop_drive();
                return;
            }
            delay(0, 0, 0, LOOP_DELAY_MS);
            elapsed += LOOP_DELAY_MS;
        }
        stop_drive();
        return;
    }

    update_speed_ramp();
    speed = motor_speed();

    if (left && center) {
        LMotor_1(0);
        RMotor_1(speed);
    } else if (center && right) {
        LMotor_1(speed);
        RMotor_1(0);
    } else if (left) {
        LMotor_1(0);
        RMotor_1(speed);
    } else if (right) {
        LMotor_1(speed);
        RMotor_1(0);
    } else {
        drive_forward_ramped();
    }
}

/* Fixed low-speed crossing pass; timings must be calibrated on the real course. */
static unsigned char clear_first_crossing(void)
{
    unsigned int elapsed = 0;

    LMotor_1(CROSSING_SPEED);
    RMotor_1(CROSSING_SPEED);
    delay(0, 0, 0, CROSSING_PASS_MS);
    elapsed += CROSSING_PASS_MS;

    while (elapsed < CROSSING_MAX_MS) {
        if (!all_black()) {
            stop_drive();
            delay(0, 0, 0, 40);
            current_speed = 0.0f;
            target_speed = 0.0f;
            return 1;
        }
        LMotor_1(CROSSING_SPEED);
        RMotor_1(CROSSING_SPEED);
        delay(0, 0, 0, CROSSING_SAMPLE_MS);
        elapsed += CROSSING_SAMPLE_MS;
    }

    stop_drive();
    current_speed = 0.0f;
    target_speed = 0.0f;
    return 0;
}

static void manual_remote_mode(void)
{
    for (;;) {
        if (rc(DOWN_KEY) == TRUE) {
            /* DOWN has priority: brake/decelerate, never reverse. */
            target_speed = 0.0f;
        } else if (rc(UP_KEY) == TRUE) {
            target_speed = SPEED_STRAIGHT;
        } else if (rc(LEFT_KEY) == TRUE || rc(RIGHT_KEY) == TRUE) {
            /* Preserve steering-only input; turn at a moderate target speed. */
            target_speed = SPEED_MEDIUM;
        } else {
            /* Releasing all controls lets the robot ramp down to a stop. */
            target_speed = 0.0f;
        }

        update_speed_ramp();

        if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(motor_speed());
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(motor_speed());
            RMotor_1(0);
        } else {
            drive_forward_ramped();
        }

        delay(0, 0, 0, LOOP_DELAY_MS);
    }
}

int main(void)
{
    unsigned char stage = STAGE_FIRST;

    mcu_initialization();
    stop_drive();

    for (;;) {
        if (all_black() && crossing_stable()) {
            stop_drive();

            if (stage == STAGE_FIRST) {
                if (!clear_first_crossing()) {
                    for (;;) {
                        stop_drive();
                        delay(0, 0, 0, 100);
                    }
                }
                stage = STAGE_SECOND;
            } else {
                stop_drive();
                delay(0, 0, 0, 250);
                current_speed = 0.0f;
                target_speed = 0.0f;
                manual_remote_mode();
            }
        } else {
            follow_line_adaptive();
        }

        delay(0, 0, 0, LOOP_DELAY_MS);
    }

    return 0;
}
