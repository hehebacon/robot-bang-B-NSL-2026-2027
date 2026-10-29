/*
 * Huna / E-Robot Bảng B firmware example.
 * Requires the platform-provided functions_codes.h.
 *
 * Sensor convention follows the supplied example:
 * ir(sensor, 0, NONE) == TRUE means that sensor detects the black line.
 * Positive motor speed = forward; negative = reverse; zero = stop.
 */
#include "functions_codes.h"

#define MOTOR_SPEED 5
#define CROSSING_DRIVE_MS 250

static unsigned char line_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static void stop_motors(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static void follow_line(void)
{
    unsigned char left_black = line_black(1);
    unsigned char center_black = line_black(2);
    unsigned char right_black = line_black(3);

    /* All-black is handled by the stage loops. */
    if (left_black && center_black && right_black) {
        stop_motors();
    } else if (left_black && center_black) {
        /* IR1 + IR2: turn left. */
        LMotor_1(0);
        RMotor_1(MOTOR_SPEED);
    } else if (center_black && right_black) {
        /* IR2 + IR3: turn right. */
        LMotor_1(MOTOR_SPEED);
        RMotor_1(0);
    } else if (left_black && !center_black && !right_black) {
        LMotor_1(0);
        RMotor_1(MOTOR_SPEED);
    } else if (right_black && !center_black && !left_black) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(0);
    } else if (center_black) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    } else {
        /* No sensor sees black: stop instead of retaining an old command. */
        stop_motors();
    }
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-MOTOR_SPEED);
            RMotor_1(-MOTOR_SPEED);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(MOTOR_SPEED);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(0);
        } else {
            /* OFF_KEY or no recognized key: stop safely. */
            stop_motors();
        }
    }
}

int main(void)
{
    mcu_initialization();
    stop_motors();

    /* Stage 1: follow the line; at the first all-black crossing,
       drive forward briefly and then proceed to stage 2. */
    for (;;) {
        unsigned char left_black = line_black(1);
        unsigned char center_black = line_black(2);
        unsigned char right_black = line_black(3);

        if (left_black && center_black && right_black) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
            delay(0, 0, 0, CROSSING_DRIVE_MS);
            stop_motors();
            break;
        }

        follow_line();
    }

    /* Stage 2: follow the line until the second all-black crossing,
       then stop and enter remote control. */
    for (;;) {
        unsigned char left_black = line_black(1);
        unsigned char center_black = line_black(2);
        unsigned char right_black = line_black(3);

        if (left_black && center_black && right_black) {
            stop_motors();
            delay(0, 0, 0, CROSSING_DRIVE_MS);
            break;
        }

        follow_line();
    }

    remote_control();
    return 0;
}
