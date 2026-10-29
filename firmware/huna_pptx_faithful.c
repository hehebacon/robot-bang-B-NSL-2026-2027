/*
 * Huna / E-Robot V1.4 — Bảng B
 * Bản viết lại bám theo tài liệu "HƯỚNG DẪN LẬP TRÌNH BẢNG B.pptx".
 *
 * Lưu ý:
 * - Theo slide: cảm biến gặp vạch đen => trạng thái NONE.
 * - Motor speed > 0: tiến; < 0: lùi; = 0: dừng.
 * - IR1 = trái, IR2 = giữa, IR3 = phải (vị trí khuyến nghị trong slide).
 * - MOTOR_SPEED và CROSSING_DRIVE_MS cần căn chỉnh theo robot thực tế.
 */

#include "functions_codes.h"

#define MOTOR_SPEED 5
#define CROSSING_DRIVE_MS 250

static unsigned char line_black(unsigned char sensor)
{
    /* Theo API mẫu Huna: yêu cầu trạng thái NONE và kiểm tra TRUE. */
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char all_three_black(void)
{
    return line_black(1) && line_black(2) && line_black(3);
}

static void stop_motors(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static void drive_forward(void)
{
    LMotor_1(MOTOR_SPEED);
    RMotor_1(MOTOR_SPEED);
}

static void drive_backward(void)
{
    LMotor_1(-MOTOR_SPEED);
    RMotor_1(-MOTOR_SPEED);
}

/* Đúng chiều rẽ theo slide:
 * trái: motor phải tiến, motor trái lùi;
 * phải: motor phải lùi, motor trái tiến.
 */
static void turn_left(void)
{
    LMotor_1(-MOTOR_SPEED);
    RMotor_1(MOTOR_SPEED);
}

static void turn_right(void)
{
    LMotor_1(MOTOR_SPEED);
    RMotor_1(-MOTOR_SPEED);
}

static void follow_line(void)
{
    unsigned char ir1 = line_black(1);
    unsigned char ir2 = line_black(2);
    unsigned char ir3 = line_black(3);

    /* Vạch ngang được xử lý riêng ở từng giai đoạn. */
    if (ir1 && ir2 && ir3) {
        stop_motors();
    }
    /* Ưu tiên góc vuông trước điều kiện cảm biến đơn. */
    else if (ir1 && ir2) {
        turn_left();
    }
    else if (ir2 && ir3) {
        turn_right();
    }
    else if (ir1) {
        turn_left();
    }
    else if (ir3) {
        turn_right();
    }
    else if (ir2) {
        drive_forward();
    }
    else {
        /* Tài liệu không chỉ định hành vi khi mất line: chọn dừng an toàn. */
        stop_motors();
    }
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            drive_forward();
        }
        else if (rc(DOWN_KEY) == TRUE) {
            drive_backward();
        }
        else if (rc(LEFT_KEY) == TRUE) {
            turn_left();
        }
        else if (rc(RIGHT_KEY) == TRUE) {
            turn_right();
        }
        else if (rc(OFF_KEY) == TRUE) {
            /* KEY OFF: dừng hai motor theo slide. */
            stop_motors();
        }
        else {
            /* Không có nút điều khiển: dừng theo yêu cầu KEY OFF của slide. */
            stop_motors();
        }
    }
}

int main(void)
{
    mcu_initialization();
    stop_motors();

    /*
     * GIAI ĐOẠN 1:
     * Bám line cho tới vạch ngang lần 1 (IR1 + IR2 + IR3 đều NONE/đen).
     * Chạy thẳng theo thời gian rồi chuyển sang giai đoạn 2.
     */
    for (;;) {
        if (all_three_black()) {
            drive_forward();
            delay(0, 0, 0, CROSSING_DRIVE_MS);
            stop_motors();

            /*
             * Chờ robot rời vạch ngang thứ nhất để không nhận nhầm
             * chính vạch đó là vạch ngang lần 2 ở đầu giai đoạn kế tiếp.
             */
            while (all_three_black()) {
                stop_motors();
                delay(0, 0, 0, 10);
            }
            break;
        }

        follow_line();
    }

    /*
     * GIAI ĐOẠN 2:
     * Bám line tương tự giai đoạn 1; vạch ngang lần 2 thì dừng
     * và chuyển sang điều khiển remote.
     */
    for (;;) {
        if (all_three_black()) {
            stop_motors();
            delay(0, 0, 0, 250);
            break;
        }

        follow_line();
    }

    /* GIAI ĐOẠN 3: điều khiển remote; KEY OFF/không có nút => dừng. */
    remote_control();

    return 0;
}
