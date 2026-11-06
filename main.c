/*
 * BANG B - HUNA E-ROBOT
 *
 * Chassis: 3 front ground IR sensors (left/center/right), two driven rear
 * wheels, two passive front wheels.
 *
 * Black-line convention follows the comments in the supplied Huna template:
 * ir(channel, 0, NONE) == TRUE.
 *
 * No forward obstacle sensor or wheel encoder is confirmed. This controller
 * therefore follows the floor line only; it cannot detect obstacles or
 * measure actual wheel RPM.
 *
 * Huna delay() time units have not been verified, so this file does not use
 * delay(). Corner recovery is stateful and uses sensor feedback, not a
 * guessed millisecond duration.
 */
#include "functions_codes.h"

#define MOTOR_LIMIT                 5
#define STRAIGHT_SPEED              3
#define SOFT_TURN_OUTER_SPEED       4
#define SOFT_TURN_INNER_SPEED       1
#define PIVOT_OUTER_SPEED           3
#define PIVOT_INNER_STOP            0
#define CROSSING_CONFIRM_COUNT      4
#define CROSSING_CLEAR_GUARD        100000UL
#define CROSSING_PASS_SPEED         2

#define REMOTE_MAX_SPEED            5
#define REMOTE_TURN_DELTA           2

#define LEFT_MOTOR_TRIM_PERCENT   100
#define RIGHT_MOTOR_TRIM_PERCENT  100

#define STAGE_FIRST                 0
#define STAGE_SECOND                1

/* -1 = last detected line was left, +1 = right, 0 = not yet known. */
static int last_turn_direction = 0;
/* 0 = normal tracking, -1 = committed left 90-degree turn, +1 = right. */
static int corner_turn_direction = 0;
static unsigned long line_lost_count = 0;

static int clamp_motor_speed(int speed)
{
    if (speed > MOTOR_LIMIT) return MOTOR_LIMIT;
    if (speed < -MOTOR_LIMIT) return -MOTOR_LIMIT;
    return speed;
}

static int apply_motor_trim(int command, int trim_percent)
{
    int scaled = command * trim_percent;
    if (scaled >= 0) scaled = (scaled + 50) / 100;
    else scaled = (scaled - 50) / 100;
    return clamp_motor_speed(scaled);
}

static void set_motor_speeds(int left_command, int right_command)
{
    LMotor_1(apply_motor_trim(left_command, LEFT_MOTOR_TRIM_PERCENT));
    RMotor_1(apply_motor_trim(right_command, RIGHT_MOTOR_TRIM_PERCENT));
}

static void motors_stop(void)
{
    set_motor_speeds(0, 0);
}

static unsigned char ir_black(unsigned char channel)
{
    return (ir(channel, 0, NONE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return (ir_black(1) && ir_black(2) && ir_black(3));
}

/*
 * Sensor table:
 * 010: centered line -> straight, deliberately moderate speed.
 * 110 / 011: curve -> slow inner wheel, keep outer wheel moving.
 * 100 / 001: sharp corner -> pivot toward the detected line until center
 *            sensor reacquires it.
 * 000: line temporarily lost -> search in the last known turn direction without reversing either wheel.
 * 101: both outer sensors active -> treat as a broad/ambiguous patch and
 *      creep straight; do not make an arbitrary turn.
 *
 * This avoids the old behavior of stopping immediately on 000 and losing
 * all information about which way the robot should search.
 */
static void follow_line(unsigned char left_black,
                        unsigned char center_black,
                        unsigned char right_black)
{
    /*
     * Once an outer-only sensor detects a sharp corner, commit to that
     * direction. Do not alternate between pivot and curve corrections while
     * the sensor pattern changes during the turn. Release the turn only when
     * the center sensor reacquires the black line.
     */
    if (corner_turn_direction < 0) {
        if (center_black) {
            corner_turn_direction = 0;
            line_lost_count = 0;
            set_motor_speeds(STRAIGHT_SPEED, STRAIGHT_SPEED);
        } else {
            set_motor_speeds(PIVOT_INNER_STOP, PIVOT_OUTER_SPEED);
        }
        return;
    }

    if (corner_turn_direction > 0) {
        if (center_black) {
            corner_turn_direction = 0;
            line_lost_count = 0;
            set_motor_speeds(STRAIGHT_SPEED, STRAIGHT_SPEED);
        } else {
            set_motor_speeds(PIVOT_OUTER_SPEED, PIVOT_INNER_STOP);
        }
        return;
    }

    /* An all-black patch is handled by the course-stage logic in main(). */
    if (left_black && center_black && right_black) {
        motors_stop();
        return;
    }

    /* Centered line. */
    if (!left_black && center_black && !right_black) {
        line_lost_count = 0;
        set_motor_speeds(STRAIGHT_SPEED, STRAIGHT_SPEED);
        return;
    }

    /* Gradual left/right curves: preserve forward motion on both wheels. */
    if (left_black && center_black && !right_black) {
        last_turn_direction = -1;
        line_lost_count = 0;
        set_motor_speeds(SOFT_TURN_INNER_SPEED, SOFT_TURN_OUTER_SPEED);
        return;
    }

    if (!left_black && center_black && right_black) {
        last_turn_direction = 1;
        line_lost_count = 0;
        set_motor_speeds(SOFT_TURN_OUTER_SPEED, SOFT_TURN_INNER_SPEED);
        return;
    }

    /* Sharp left/right corner: latch the turn until center sees the line. */
    if (left_black && !center_black && !right_black) {
        last_turn_direction = -1;
        corner_turn_direction = -1;
        line_lost_count = 0;
        set_motor_speeds(PIVOT_INNER_STOP, PIVOT_OUTER_SPEED);
        return;
    }

    if (!left_black && !center_black && right_black) {
        last_turn_direction = 1;
        corner_turn_direction = 1;
        line_lost_count = 0;
        set_motor_speeds(PIVOT_OUTER_SPEED, PIVOT_INNER_STOP);
        return;
    }

    /* No sensor sees the line: search in the last known direction. */
    if (!left_black && !center_black && !right_black) {
        ++line_lost_count;

        if (last_turn_direction < 0) {
            set_motor_speeds(PIVOT_INNER_STOP, PIVOT_OUTER_SPEED);
        } else if (last_turn_direction > 0) {
            set_motor_speeds(PIVOT_OUTER_SPEED, PIVOT_INNER_STOP);
        } else {
            motors_stop();
        }

        if (line_lost_count > CROSSING_CLEAR_GUARD) {
            motors_stop();
            last_turn_direction = 0;
            corner_turn_direction = 0;
            line_lost_count = 0;
        }
        return;
    }

    /* Outer sensors only: ambiguous wide marking; move forward slowly. */
    if (left_black && !center_black && right_black) {
        line_lost_count = 0;
        set_motor_speeds(2, 2);
        return;
    }

    motors_stop();
}

/*
 * Latched manual control:
 * DOWN priority; UP+LEFT/RIGHT makes a forward arc.
 * LEFT alone = (0, 5), RIGHT alone = (5, 0), as previously specified.
 * OFF_KEY and no key both stop immediately.
 */
static void remote_control(void)
{
    for (;;) {
        unsigned char off = (rc(OFF_KEY) == TRUE);
        unsigned char up = (rc(UP_KEY) == TRUE);
        unsigned char down = (rc(DOWN_KEY) == TRUE);
        unsigned char left = (rc(LEFT_KEY) == TRUE);
        unsigned char right = (rc(RIGHT_KEY) == TRUE);
        int target_left = 0;
        int target_right = 0;

        if (off) {
            motors_stop();
        } else if (down) {
            set_motor_speeds(-REMOTE_MAX_SPEED, -REMOTE_MAX_SPEED);
        } else if (up) {
            target_left = REMOTE_MAX_SPEED;
            target_right = REMOTE_MAX_SPEED;

            if (left && !right) {
                target_left -= REMOTE_TURN_DELTA;
            } else if (right && !left) {
                target_right -= REMOTE_TURN_DELTA;
            }
            set_motor_speeds(target_left, target_right);
        } else if (left && !right) {
            set_motor_speeds(0, REMOTE_MAX_SPEED);
        } else if (right && !left) {
            set_motor_speeds(REMOTE_MAX_SPEED, 0);
        } else {
            motors_stop();
        }
    }
}

int main(void)
{
    unsigned char stage = STAGE_FIRST;
    unsigned char crossing_count = 0;

    mcu_initialization();
    motors_stop();

    for (;;) {
        unsigned char left_black = ir_black(1);
        unsigned char center_black = ir_black(2);
        unsigned char right_black = ir_black(3);
        unsigned char crossing =
            (left_black && center_black && right_black);

        if (crossing) {
            if (crossing_count < CROSSING_CONFIRM_COUNT) {
                ++crossing_count;
            }
        } else {
            crossing_count = 0;
        }

        if (crossing_count >= CROSSING_CONFIRM_COUNT) {
            if (stage == STAGE_FIRST) {
                unsigned long guard = 0;

                /* Pass the first all-black marker at low speed. */
                while (all_three_black() &&
                       guard < CROSSING_CLEAR_GUARD) {
                    set_motor_speeds(CROSSING_PASS_SPEED,
                                     CROSSING_PASS_SPEED);
                    ++guard;
                }

                motors_stop();

                if (all_three_black()) {
                    /* Sensor pattern did not clear: fail stopped. */
                    for (;;) {
                        motors_stop();
                    }
                }

                stage = STAGE_SECOND;
                crossing_count = 0;
                last_turn_direction = 0;
                line_lost_count = 0;
            } else {
                motors_stop();
                remote_control();
            }
        } else {
            follow_line(left_black, center_black, right_black);
        }
    }

    return 0;
}
