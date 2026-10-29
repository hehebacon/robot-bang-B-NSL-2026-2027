/*
 * Huna / E-Robot V1.4 — Bảng B
 * Chỉnh luồng AUTO -> AUTO giai đoạn 2 -> MANUAL dựa trên source mẫu
 * người dùng cung cấp.
 *
 * Giữ nguyên quy ước gọi ir() và các tổ hợp cảm biến của source mẫu.
 * Chưa xác minh chiều motor/cảm biến trên phần cứng thật.
 */
#include "functions_codes.h"

#define MOTOR_SPEED 5
#define CROSSING_DRIVE_MS 250
#define CROSSING_CLEAR_TIMEOUT_TICKS 500 /* 500 x 10 ms = 5 seconds */

static void stop_motors(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char all_three_none(void)
{
    return (ir(1, 0, NONE) == TRUE &&
            ir(2, 0, NONE) == TRUE &&
            ir(3, 0, NONE) == TRUE);
}

/*
 * Giữ nguyên các pattern và lệnh motor từ source mẫu.
 * Nhánh trùng NONE/SENSE/SENSE đã bỏ vì nó không thể phân biệt
 * với nhánh giống hệt đứng trước.
 */
static void follow_line_sample(void)
{
    if (ir(1, 0, NONE) == TRUE &&
        ir(2, 0, NONE) == TRUE &&
        ir(3, 0, NONE) == TRUE) {
        /* Vạch ngang được xử lý bởi vòng AUTO, không xử lý ở đây. */
        stop_motors();
    }
    else if (ir(1, 0, NONE) == TRUE &&
             ir(2, 0, NONE) == TRUE &&
             ir(3, 950, SENSE) == TRUE) {
        LMotor_1(0);
        RMotor_1(MOTOR_SPEED);
    }
    else if (ir(1, 0, NONE) == TRUE &&
             ir(2, 950, SENSE) == TRUE &&
             ir(3, 950, SENSE) == TRUE) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(0);
    }
    else if (ir(1, 950, SENSE) == TRUE &&
             ir(2, 950, SENSE) == TRUE &&
             ir(3, 0, NONE) == TRUE) {
        LMotor_1(0);
        RMotor_1(MOTOR_SPEED);
    }
    else if (ir(1, 950, SENSE) == TRUE &&
             ir(2, 0, NONE) == TRUE &&
             ir(3, 950, SENSE) == TRUE) {
        LMotor_1(MOTOR_SPEED);
        RMotor_1(MOTOR_SPEED);
    }
    else {
        /* Trạng thái không được source mẫu mô tả: dừng thay vì giữ lệnh cũ. */
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
        else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(0);
        }
        else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(MOTOR_SPEED);
        }
        else {
            /* OFF_KEY hoặc không có phím: dừng. */
            stop_motors();
        }
    }
}

int main(void)
{
    unsigned int clear_wait_ticks;

    mcu_initialization();
    stop_motors();

    /*
     * AUTO giai đoạn 1:
     * Tự dò line cho tới giao điểm đầu tiên.
     */
    for (;;) {
        if (all_three_none()) {
            LMotor_1(MOTOR_SPEED);
            RMotor_1(MOTOR_SPEED);
            delay(0, 0, 0, CROSSING_DRIVE_MS);

            /*
             * Chờ rời vạch ngang để giai đoạn 2 không nhận nhầm
             * chính giao điểm đầu tiên thành giao điểm thứ hai.
             */
            clear_wait_ticks = 0;
            while (all_three_none() &&
                   clear_wait_ticks < CROSSING_CLEAR_TIMEOUT_TICKS) {
                LMotor_1(MOTOR_SPEED);
                RMotor_1(MOTOR_SPEED);
                delay(0, 0, 0, 10);
                ++clear_wait_ticks;
            }

            if (all_three_none()) {
                /* Không rời được trạng thái giao điểm: dừng an toàn. */
                stop_motors();
                for (;;) {
                    stop_motors();
                    delay(0, 0, 0, 100);
                }
            }

            stop_motors();
            break;
        }

        follow_line_sample();
    }

    /*
     * AUTO giai đoạn 2:
     * Tiếp tục dò line; giao điểm thứ hai thì dừng và chuyển MANUAL.
     */
    for (;;) {
        if (all_three_none()) {
            stop_motors();
            delay(0, 0, 0, 250);
            break;
        }

        follow_line_sample();
    }

    /* MANUAL: điều khiển bằng remote; không có phím thì dừng. */
    manual_control();

    return 0;
}
