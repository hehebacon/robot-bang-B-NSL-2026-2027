/*
 * Huna / E-Robot V1.4 — Bảng B
 * AUTO chặng 1 -> giao điểm 1 -> AUTO chặng 2 -> MANUAL.
 *
 * Quy ước lấy từ hướng dẫn Bảng B:
 * - IR đen/vạch: ir(pin, 0, NONE) == TRUE.
 * - IR1 đen: rẽ trái; IR2 đen: đi thẳng; IR3 đen: rẽ phải.
 * - IR1+IR2: trái; IR2+IR3: phải.
 * - Cả ba IR đen: nhận diện giao điểm.
 *
 * Chưa xác minh trên phần cứng thật. Cần tinh chỉnh tốc độ và thời gian
 * chạy qua giao điểm theo robot/sân thực tế.
 */
#include "functions_codes.h"

#define MOTOR_SPEED 5
#define CROSSING_DRIVE_MS 250
#define CROSSING_CLEAR_TIMEOUT_TICKS 500 /* 500 x 10 ms = 5 s */

static void stop_motors(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

/* Theo tài liệu Huna: trạng thái NONE ở ngưỡng 0 là vạch đen. */
static unsigned char sees_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return (sees_black(1) && sees_black(2) && sees_black(3));
}

/*
 * Điều khiển line theo vị trí vạch:
 * IR1 ở trái, IR2 ở giữa, IR3 ở phải.
 * Rẽ trái: motor trái lùi, motor phải tiến.
 * Rẽ phải: motor trái tiến, motor phải lùi.
 */
static void follow_line(void)
{
    unsigned char left_black = sees_black(1);
    unsigned char center_black = sees_black(2);
    unsigned char right_black = sees_black(3);

    if (left_black && center_black && right_black) {
        /* Giao điểm được xử lý bởi state machine trong main(). */
        stop_motors();
    }
    else if (left_black && center_black) {
        LMotor_1(-MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (center_black && right_black) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(-MOTOR_SPEED);
    }
    else if (left_black) {
        LMotor_1(-MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (center_black) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else if (right_black) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(-MOTOR_SPEED);
    }
    else {
        /* Mất line / tổ hợp chưa được định nghĩa: dừng an toàn. */
        stop_motors();
    }
}

static void manual_control(void)
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
        else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(-MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
        }
        else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(-MOTOR_SPEED);
        }
        else {
            /* OFF_KEY hoặc không có phím: dừng. */
            stop_motors();
        }
    }
}

static void wait_until_crossing_clears(void)
{
    unsigned int ticks = 0;

    while (all_three_black() && ticks < CROSSING_CLEAR_TIMEOUT_TICKS) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
        delay(0, 0, 0, 10);
        ++ticks;
    }

    if (all_three_black()) {
        /* Không thoát được vùng giao điểm: không tự chạy tiếp mù. */
        stop_motors();
        for (;;) {
            stop_motors();
            delay(0, 0, 0, 100);
        }
    }
}

int main(void)
{
    mcu_initialization();
    stop_motors();

    /* AUTO chặng 1: dò line tới giao điểm thứ nhất. */
    for (;;) {
        if (all_three_black()) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
            delay(0, 0, 0, CROSSING_DRIVE_MS);
            wait_until_crossing_clears();
            stop_motors();
            break;
        }

        follow_line();
    }

    /* AUTO chặng 2: giao điểm tiếp theo thì dừng và chuyển MANUAL. */
    for (;;) {
        if (all_three_black()) {
            stop_motors();
            delay(0, 0, 0, 250);
            break;
        }

        follow_line();
    }

    manual_control();
    return 0;
}
