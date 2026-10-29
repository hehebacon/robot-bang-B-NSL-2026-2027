#pragma once

#include <cstdint>

namespace robot_b {

enum class Mode : std::uint8_t { Autonomous, Remote };
enum class RemoteKey : std::uint8_t { Off, Forward, Backward, Left, Right };

struct Input {
  // true means the sensor detects the black line. The E-Robot guide describes
  // black as "None"; verify the actual module's electrical polarity on hardware.
  bool ir_left_black{false};
  bool ir_center_black{false};
  bool ir_right_black{false};
  RemoteKey remote_key{RemoteKey::Off};
  std::uint32_t now_ms{0};
};

struct Output {
  // Signed motor command: positive = forward, negative = reverse, zero = stop.
  // The board-specific adapter must translate these values to the real driver.
  int left_motor{0};
  int right_motor{0};
  Mode mode{Mode::Autonomous};
  unsigned stage{1};
};

struct Config {
  int forward_speed{55};
  int turn_speed{45};
  std::uint32_t first_cross_drive_ms{350};
};

class Controller {
 public:
  explicit Controller(Config config = {}) : config_(config) {}

  Output update(const Input& in) {
    if (mode_ == Mode::Remote) {
      return remote_output(in.remote_key);
    }

    const bool all_black =
        in.ir_left_black && in.ir_center_black && in.ir_right_black;

    if (stage_ == 1 && crossing_) {
      if (elapsed(in.now_ms, crossing_started_ms_) >= config_.first_cross_drive_ms) {
        crossing_ = false;
        stage_ = 2;
        return stopped();
      }
      return drive(config_.forward_speed, config_.forward_speed);
    }

    if (stage_ == 1 && all_black) {
      crossing_ = true;
      crossing_started_ms_ = in.now_ms;
      return drive(config_.forward_speed, config_.forward_speed);
    }

    if (stage_ == 2 && all_black) {
      mode_ = Mode::Remote;
      return stopped();
    }

    // Follow the guide's sensor cases: center=line -> straight; left=line ->
    // turn left; right=line -> turn right. Combined adjacent sensors handle
    // right-angle corners. These are signed motor commands, not GPIO values.
    if (in.ir_left_black && in.ir_center_black) {
      return drive(-config_.turn_speed, config_.forward_speed);
    }
    if (in.ir_center_black && in.ir_right_black) {
      return drive(config_.forward_speed, -config_.turn_speed);
    }
    if (in.ir_center_black) {
      return drive(config_.forward_speed, config_.forward_speed);
    }
    if (in.ir_left_black) {
      return drive(-config_.turn_speed, config_.turn_speed);
    }
    if (in.ir_right_black) {
      return drive(config_.turn_speed, -config_.turn_speed);
    }

    // No sensor sees the line: fail safe and stop rather than drive blindly.
    return stopped();
  }

  void reset() {
    mode_ = Mode::Autonomous;
    stage_ = 1;
    crossing_ = false;
    crossing_started_ms_ = 0;
  }

 private:
  static std::uint32_t elapsed(std::uint32_t now, std::uint32_t start) {
    return now - start;  // unsigned subtraction safely handles millis() wrap.
  }

  Output drive(int left, int right) const {
    return Output{left, right, mode_, stage_};
  }

  Output stopped() const { return drive(0, 0); }

  Output remote_output(RemoteKey key) const {
    switch (key) {
      case RemoteKey::Forward:
        return drive(config_.forward_speed, config_.forward_speed);
      case RemoteKey::Backward:
        return drive(-config_.forward_speed, -config_.forward_speed);
      case RemoteKey::Left:
        return drive(-config_.turn_speed, config_.turn_speed);
      case RemoteKey::Right:
        return drive(config_.turn_speed, -config_.turn_speed);
      case RemoteKey::Off:
      default:
        return stopped();
    }
  }

  Config config_;
  Mode mode_{Mode::Autonomous};
  unsigned stage_{1};
  bool crossing_{false};
  std::uint32_t crossing_started_ms_{0};
};

}  // namespace robot_b
