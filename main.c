/*
 * BANG B - HUNA E-ROBOT V1.4
 * Three-IR differential-drive line follower with integer steering correction.
 *
 * Confirmed current chassis layout from the user:
 * - IR1 / IR2 / IR3 are arranged left / center / right in one horizontal row
 *   at the front of the robot;
 * - rear left/right wheels are driven independently;
 * - front wheels are passive.
 *
 * Sensor/API assumptions retained from the existing Huna entry point:
 *   ir(channel, 950, SENSE) == TRUE means black line.
 *   Positive motor speed means forward; zero means stop.
 * Verify these conventions on the real board before a track run.
 *
 * No forward obstacle/range sensor is installed. This firmware therefore
 * does NOT claim to detect or avoid physical obstacles. It only follows the
 * line, handles the configured all-three-black crossing pattern, and offers
 * the existing latched manual remote mode.
 *
 * Source-level update only: proprietary Huna compile, DOWNLOAD, and physical
 * boot have not been performed or verified.
 */
#include "functions_codes.h"

#define LOOP_DELAY_MS                 10
#define CROSSING_DRIVE_MS            180
#define CROSSING_CLEAR_TIMEOUT_MS   1500
#define CROSSING_CONFIRM_COUNT         4

#define LINE_MAX_SPEED                 5
#define LINE_MIN_SPEED                 1
#define LINE_KP                        1
#define LINE_KD                        1
#define MOTOR_OUTPUT_LIMIT             5

#define STAGE_FIRST                    0
#define STAGE_SECOND                   1

#define REMOTE_TURN_DELTA              2
#define REMOTE_MAX_SPEED               5
#define REMOTE_RAMP_INTERVAL_MS       30

static int remote_left_current = 0;
static int remote_right_current = 0;
static int remote_ramp_elapsed_ms = 0;
static int previous_line_error = 0;
static unsigned char has_previous_line_error = 0;

/*
 * Independent fixed trims compensate a stable left/right motor mismatch.
 * 100 means unchanged. These are NOT live speed feedback: no wheel encoders
 * are confirmed on this robot. Tune only after controlled, safe testing.
 */
#define LEFT_MOTOR_TRIM_PERCENT       100
#define RIGHT_MOTOR_TRIM_PERCENT      100

static int clamp_motor_speed(int speed)
{
    if (speed > MOTOR_OUTPUT_LIMIT) return MOTOR_OUTPUT_LIMIT;
    if (speed < -MOTOR_OUTPUT_LIMIT) return -MOTOR_OUTPUT_LIMIT;
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
    return (ir(channel, 950, SENSE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return (ir_black(1) && ir_black(2) && ir_black(3));
}

/*
 * Adaptive integer PD line controller.
 * Sensor position gives the error (-2..+2); change since the previous sample
 * adds derivative damping. Forward speed is reduced as the line moves away
 * from the center sensor. This is NOT wheel-RPM regulation: no encoders are
 * confirmed, and binary sensors limit control precision.
 */
static void follow_line(unsigned char left_black,
                        unsigned char center_black,
                        unsigned char right_black)
{
    int error = 0;
    int derivative = 0;
    int correction;
    int base_speed;
    int left_command;
    int right_command;
    int abs_error;

    if (left_black && center_black && right_black) {
        has_previous_line_error = 0;
        motors_stop();
        return;
    }
    if (!left_black && !center_black && !right_black) {
        has_previous_line_error = 0;
        motors_stop();
        return;
    }
    if (left_black && !center_black && right_black) {
        has_previous_line_error = 0;
        motors_stop();
        return;
    }

    if (left_black && center_black) error = -1;
    else if (left_black) error = -2;
    else if (center_black && right_black) error = 1;
    else if (right_black) error = 2;
    else error = 0;

    if (has_previous_line_error) {
        derivative = error - previous_line_error;
    }
    previous_line_error = error;
    has_previous_line_error = 1;

    abs_error = (error < 0) ? -error : error;
    base_speed = LINE_MAX_SPEED - abs_error;
    if (base_speed < 2) base_speed = 2;

    correction = (LINE_KP * error) + (LINE_KD * derivative);
    left_command = base_speed + correction;
    right_command = base_speed - correction;

    if (left_command < LINE_MIN_SPEED) left_command = LINE_MIN_SPEED;
    if (right_command < LINE_MIN_SPEED) right_command = LINE_MIN_SPEED;
    if (left_command > LINE_MAX_SPEED) left_command = LINE_MAX_SPEED;
    if (right_command > LINE_MAX_SPEED) right_command = LINE_MAX_SPEED;

    set_motor_speeds(left_command, right_command);
}

static int clamp_remote_speed(int speed)
{
    if (speed > REMOTE_MAX_SPEED) return REMOTE_MAX_SPEED;
    if (speed < -REMOTE_MAX_SPEED) return -REMOTE_MAX_SPEED;
    return speed;
}

static int approach_remote_target(int current, int target)
{
    if (current < target) return current + 1;
    if (current > target) return current - 1;
    return current;
}

/*
 * Manual mode is latched, as in the previous version.
 * UP/DOWN + LEFT/RIGHT make differential-drive arcs.
 * LEFT/RIGHT alone pivot; UP/DOWN alone drive straight.
 * Simultaneous key reporting depends on the Huna rc() implementation and
 * still needs confirmation on the physical controller.
 */
/*
 * Manual mode stays latched. DOWN has priority over steering; UP enables
 * forward motion and can combine with LEFT/RIGHT for an arc. LEFT or RIGHT
 * alone use the supplied one-wheel-forward mapping. OFF_KEY and no-key both
 * stop the motors.
 */
static void remote_control(void)
{
    remote_left_current = 0;
    remote_right_current = 0;
    remote_ramp_elapsed_ms = 0;

    for (;;) {
        unsigned char up = (rc(UP_KEY) == TRUE);
        unsigned char down = (rc(DOWN_KEY) == TRUE);
        unsigned char left = (rc(LEFT_KEY) == TRUE);
        unsigned char right = (rc(RIGHT_KEY) == TRUE);
        unsigned char off = (rc(OFF_KEY) == TRUE);
        int target_left = 0;
        int target_right = 0;

        if (off) {
            target_left = 0;
            target_right = 0;
            remote_left_current = 0;
            remote_right_current = 0;
            remote_ramp_elapsed_ms = 0;
        } else if (down) {
            target_left = -REMOTE_MAX_SPEED;
            target_right = -REMOTE_MAX_SPEED;
        } else if (up) {
            target_left = REMOTE_MAX_SPEED;
            target_right = REMOTE_MAX_SPEED;
            if (left && !right) {
                target_left -= REMOTE_TURN_DELTA;
            } else if (right && !left) {
                target_right -= REMOTE_TURN_DELTA;
            }
        } else if (left && !right) {
            target_left = 0;
            target_right = REMOTE_MAX_SPEED;
        } else if (right && !left) {
            target_left = REMOTE_MAX_SPEED;
            target_right = 0;
        }

        target_left = clamp_remote_speed(target_left);
        target_right = clamp_remote_speed(target_right);

        if (!off && !up && !down && !left && !right) {
            /* No key means immediate stop, not a slow ramp-down. */
            remote_left_current = 0;
            remote_right_current = 0;
            remote_ramp_elapsed_ms = 0;
        } else {
            remote_ramp_elapsed_ms += LOOP_DELAY_MS;
            if (remote_ramp_elapsed_ms >= REMOTE_RAMP_INTERVAL_MS) {
                remote_ramp_elapsed_ms = 0;
                remote_left_current =
                    approach_remote_target(remote_left_current, target_left);
                remote_right_current =
                    approach_remote_target(remote_right_current, target_right);
            }
        }

        set_motor_speeds(remote_left_current, remote_right_current);
        delay(0, 0, 0, LOOP_DELAY_MS);
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
            unsigned int clear_wait_ms = 0;

            motors_stop();

            if (stage == STAGE_FIRST) {
                /* Pass the first all-three-black marker at low speed. */
                set_motor_speeds(2, 2);
                delay(0, 0, 0, CROSSING_DRIVE_MS);

                while (all_three_black() &&
                       clear_wait_ms < CROSSING_CLEAR_TIMEOUT_MS) {
                    set_motor_speeds(2, 2);
                    delay(0, 0, 0, LOOP_DELAY_MS);
                    clear_wait_ms += LOOP_DELAY_MS;
                }

                if (clear_wait_ms >= CROSSING_CLEAR_TIMEOUT_MS &&
                    all_three_black()) {
                    /* Sensor pattern did not clear: fail stopped. */
                    motors_stop();
                    for (;;) {
                        delay(0, 0, 0, 100);
                    }
                }

                motors_stop();
                crossing_count = 0;
                stage = STAGE_SECOND;
            } else {
                motors_stop();
                delay(0, 0, 0, 100);
                remote_control();
            }
        } else {
            follow_line(left_black, center_black, right_black);
            delay(0, 0, 0, LOOP_DELAY_MS);
        }
    }

    return 0;
}
