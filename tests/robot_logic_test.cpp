#include "../firmware/robot_logic.hpp"

#include <cassert>
#include <iostream>

using namespace robot_b;

int main() {
  Controller controller(Config{60, 40, 100});
  Input in{};

  auto out = controller.update(in);
  assert(out.left_motor == 0 && out.right_motor == 0);

  in.ir_center_black = true;
  out = controller.update(in);
  assert(out.left_motor == 60 && out.right_motor == 60);

  in.ir_left_black = true;
  out = controller.update(in);
  assert(out.left_motor == -40 && out.right_motor == 60);

  // First all-black cross: drive through for the configured duration.
  in.ir_left_black = true;
  in.ir_center_black = true;
  in.ir_right_black = true;
  in.now_ms = 1000;
  out = controller.update(in);
  assert(out.left_motor == 60 && out.right_motor == 60 && out.stage == 1);

  in.now_ms = 1100;
  out = controller.update(in);
  assert(out.left_motor == 0 && out.right_motor == 0 && out.stage == 2);

  // Second all-black cross switches to remote mode and stops.
  out = controller.update(in);
  assert(out.mode == Mode::Remote);
  assert(out.left_motor == 0 && out.right_motor == 0);

  in.remote_key = RemoteKey::Forward;
  out = controller.update(in);
  assert(out.left_motor == 60 && out.right_motor == 60);

  in.remote_key = RemoteKey::Left;
  out = controller.update(in);
  assert(out.left_motor == -40 && out.right_motor == 40);

  in.remote_key = RemoteKey::Off;
  out = controller.update(in);
  assert(out.left_motor == 0 && out.right_motor == 0);

  controller.reset();
  out = controller.update(in);
  assert(out.mode == Mode::Autonomous && out.stage == 1);

  std::cout << "All robot logic tests passed.\n";
}
