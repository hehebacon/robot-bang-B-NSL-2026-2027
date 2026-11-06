/*
 * BANG B - HUNA E-ROBOT
 *
 * Chassis confirmed by user:
 * - IR1 / IR2 / IR3: front ground-facing sensors, left / center / right.
 * - Two independently driven rear DC motors.
 * - Two passive front wheels.
 *
 * Sensor convention follows the comments in the supplied Huna template:
 * ir(channel, 0, NONE) == TRUE is treated as black-line detection.
 *
 * IMPORTANT:
 * - No delay() call is used here because the four delay() arguments in the
 *   supplied template have not been documented well enough to treat the last
 *   argument as milliseconds.
 * - The crossing-clear guard is an iteration bound, NOT a calibrated timer.
 * - No obstacle sensor or wheel encoder is confirmed.
 * - This source has not been compiled with the proprietary Huna toolchain or
 *   tested on the physical robot.
 */
#include "functions_codes.h"

#define LINE_MAX_SPEED             5
#define LINE_MIN_SPEED             1
#define LINE_KP                    1
#define LINE_KD                    1
#define MOTOR_OUTPUT_LIMIT         5

#define CROSSING_CONFIRM_COUNT     4
#define CROSSING_CLEAR_GUARD       100000UL
#define CROSSING_PASS_SPEED        2

#define REMOTE_MAX_SPEED           5
#define REMOTE_TURN_DELTA          2

#define LEFT_MOTOR_TRIM_PERCENT    100
#define RIGHT_MOTOR_TRIM_PERCENT   100

#define STAGE_FIRST                0
#define STAGE_SECOND               1

static int previous_line_error = 0;
static unsigned char has_previous_line_error = 0;

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
    return (ir(channel, 0, NONE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return (ir_black(1) && ir_black(2) && ir_black(3));
}

/*
 * Three-sensor integer PD-style steering.
 * This estimates line position from binary sensor states. It does NOT measure
 * or automatically balance actual wheel RPM; no encoders are confirmed.
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

    if ((left_black && center_black && right_black) ||
        (!left_black && !center_black && !right_black) ||
        (left_black && !center_black && right_black)) {
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

/*
 * Manual mode remains latched.
 * DOWN takes priority. UP+LEFT/RIGHT makes a forward arc.
 * LEFT alone = (0, 5); RIGHT alone = (5, 0), matching the user's mapping.
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
            target_left = -REMOTE_MAX_SPEED;
            target_right = -REMOTE_MAX_SPEED;
            set_motor_speeds(target_left, target_right);
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

                /*
                 * Pass the first marker slowly until the sensors no longer
                 * see all-black. The guard prevents an endless drive if the
                 * sensor pattern never clears; it is not a time measurement.
                 */
                while (all_three_black() &&
                       guard < CROSSING_CLEAR_GUARD) {
                    set_motor_speeds(CROSSING_PASS_SPEED,
                                     CROSSING_PASS_SPEED);
                    ++guard;
                }

                motors_stop();

                if (all_three_black()) {
                    /* Pattern did not clear: remain stopped. */
                    for (;;) {
                        motors_stop();
                    }
                }

                stage = STAGE_SECOND;
                crossing_count = 0;
                has_previous_line_error = 0;
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
