/*
 * Huna / E-Robot V1.4 — Bảng B
 * AUTO chặng 1 -> qua giao điểm 1 -> AUTO chặng 2 -> MANUAL.
 *
 * Theo hướng dẫn Bảng B và source Huna người dùng cung cấp:
 * ir(sensor, 0, NONE) == TRUE được dùng làm trạng thái vạch đen.
 * IR1 = trái, IR2 = giữa, IR3 = phải là giả định cần xác nhận trên robot.
 *
 * Thanh gạt gắn cố định, không có lệnh servo trong chương trình này.
 * Tốc độ/thời gian qua giao điểm là giá trị khởi đầu, chưa test phần cứng.
 */
#include "functions_codes.h"

#define MOTOR_SPEED 5
#define TURN_SPEED 5
#define CROSSING_DRIVE_MS 250
#define CROSSING_CLEAR_TIMEOUT_TICKS 500 /* 500 * 10 ms = 5 s */

static void stop_motors(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char line_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return (line_black(1) && line_black(2) && line_black(3));
}

/*
 * Signed motor commands:
 * left turn  = left motor reverse, right motor forward.
 * right turn = left motor forward, right motor reverse.
 * Verify actual motor orientation before running on the course.
 */
static void follow_line(void)
{
    unsigned char left = line_black(1);
    unsigned char center = line_black(2);
    unsigned char right = line_black(3);

    if (left && center && right) {
        /* Crossing is handled by the AUTO stage state machine. */
        stop_motors();
    }
    else if (left && center) {
        /* Left + center: bias/turn left. */
        LMotor_1(-TURN_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (center && right) {
        /* Center + right: bias/turn right. */
        LMotor_1(MOTOR_SPEED);
        RMotor_1(-TURN_SPEED);
    }
    else if (left) {
        LMotor_1(-TURN_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (center) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (right) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(-TURN_SPEED);
    }
    else {
        /* No sensor detects the line: stop rather than drive blindly. */
        stop_motors();
    }
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
        }
        else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-MOTOR_SPEED);
            RMotor_1(-MOTOR_SPEED);
        }
        else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(0);
        }
        else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(MOTOR_SPEED);
        }
        else {
            /* OFF_KEY or no recognized key: stop. */
            stop_motors();
        }
    }
}

/* Move forward off the first crossing; fail stopped if it never clears. */
static void clear_first_crossing(void)
{
    unsigned int ticks = 0;

    LMotor_1(MOTOR_SPEED);
    RMotor_1(MOTOR_SPEED);
    delay(0, 0, 0, CROSSING_DRIVE_MS);

    while (all_three_black() && ticks < CROSSING_CLEAR_TIMEOUT_TICKS) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
        delay(0, 0, 0, 10);
        ++ticks;
    }

    if (all_three_black()) {
        stop_motors();
        for (;;) {
            stop_motors();
            delay(0, 0, 0, 100);
        }
    }
    stop_motors();
}

int main(void)
{
    mcu_initialization();
    stop_motors();

    /* AUTO stage 1: line-follow until the first all-black crossing. */
    for (;;) {
        if (all_three_black()) {
            clear_first_crossing();
            break;
        }
        follow_line();
    }

    /* AUTO stage 2: follow the line until the next all-black crossing. */
    for (;;) {
        if (all_three_black()) {
            stop_motors();
            delay(0, 0, 0, 250);
            break;
        }
        follow_line();
    }

    /* MANUAL: remote control; release/no key means stop. */
    remote_control();
    return 0;
}
