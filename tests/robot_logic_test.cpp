#include "../firmware/robot_logic.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

using namespace robot_b;

static void expect_drive(const Output& out, int left, int right) {
  assert(out.left_motor == left);
  assert(out.right_motor == right);
}

int main() {
  Controller controller(Config{60, 40, 100});
  Input in{};

  // Fail-safe when no sensor sees the line.
  auto out = controller.update(in);
  expect_drive(out, 0, 0);
  assert(out.mode == Mode::Autonomous);
  assert(out.stage == 1);

  // Straight and left/right line corrections.
  in.ir_center_black = true;
  out = controller.update(in);
  expect_drive(out, 60, 60);

  in.ir_left_black = true;
  out = controller.update(in);
  expect_drive(out, -40, 60);

  in.ir_left_black = false;
  in.ir_center_black = false;
  in.ir_right_black = true;
  out = controller.update(in);
  expect_drive(out, 40, -40);

  // First all-black cross: drive through for the configured duration.
  in.ir_left_black = true;
  in.ir_center_black = true;
  in.now_ms = 1000;
  out = controller.update(in);
  expect_drive(out, 60, 60);
  assert(out.stage == 1);

  in.now_ms = 1099;
  out = controller.update(in);
  expect_drive(out, 60, 60);
  assert(out.stage == 1);

  in.now_ms = 1100;
  out = controller.update(in);
  expect_drive(out, 0, 0);
  assert(out.stage == 2);

  // Second all-black cross switches to remote mode and stops.
  out = controller.update(in);
  expect_drive(out, 0, 0);
  assert(out.mode == Mode::Remote);

  // Remote directions and neutral stop.
  in.remote_key = RemoteKey::Forward;
  out = controller.update(in);
  expect_drive(out, 60, 60);

  in.remote_key = RemoteKey::Backward;
  out = controller.update(in);
  expect_drive(out, -60, -60);

  in.remote_key = RemoteKey::Left;
  out = controller.update(in);
  expect_drive(out, -40, 40);

  in.remote_key = RemoteKey::Right;
  out = controller.update(in);
  expect_drive(out, 40, -40);

  in.remote_key = RemoteKey::Off;
  out = controller.update(in);
  expect_drive(out, 0, 0);

  // Reset returns the state machine to autonomous stage 1.
  controller.reset();
  in.ir_left_black = false;
  in.ir_center_black = false;
  in.ir_right_black = false;
  out = controller.update(in);
  expect_drive(out, 0, 0);
  assert(out.mode == Mode::Autonomous && out.stage == 1);

  // Unsigned elapsed-time subtraction should tolerate a 32-bit clock wrap.
  Controller wrap_controller(Config{50, 30, 20});
  Input wrap{};
  wrap.ir_left_black = wrap.ir_center_black = wrap.ir_right_black = true;
  wrap.now_ms = UINT32_MAX - 9U;
  out = wrap_controller.update(wrap);
  expect_drive(out, 50, 50);
  wrap.now_ms = 10U;  // 20 ms elapsed across wrap
  out = wrap_controller.update(wrap);
  expect_drive(out, 0, 0);
  assert(out.stage == 2);

  std::cout << "All robot logic tests passed.\n";
  return 0;
}
