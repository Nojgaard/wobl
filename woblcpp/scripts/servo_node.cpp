#include <iostream>
#include <vector>
#include <wobl/msg/joint.pb.h>
#include <wobl/node.hpp>
#include <wobl/real/servo_driver.hpp>

enum JointToId {
  HIP_LEFT = 0,
  HIP_RIGHT = 5,
  WHEEL_LEFT = 2,
  WHEEL_RIGHT = 6,
};

const std::vector<uint8_t> servo_ids = {HIP_LEFT, HIP_RIGHT, WHEEL_LEFT,
                                        WHEEL_RIGHT};
bool is_enabled = false;

void enable_servos(wobl::real::ServoDriver &driver, bool enable) {
  if (enable == is_enabled)
    return;

  for (u8 id : servo_ids) {
    if (!driver.enable_torque(id, enable))
      std::cerr << "[SERVO] Failed to set torque for servo with ID "
                << static_cast<int>(id) << std::endl;
  }
  std::cout << "[SERVO] Torque " << (enable ? "enabled" : "disabled")
            << " for all servos" << std::endl;
  is_enabled = enable;
}

bool initialize(wobl::real::ServoDriver &driver,
                wobl::msg::JointState &msg_state) {
  std::cout << "[SERVO] Initializing..." << std::endl;
  if (!driver.initialize()) {
    std::cerr << "[SERVO] Failed to initialize servo driver" << std::endl;
    return false;
  }

  for (u8 id : servo_ids) {
    if (!driver.ping(id)) {
      std::cerr << "[SERVO] Failed to ping servo with ID "
                << static_cast<int>(id) << std::endl;
      return false;
    }
    std::cout << "[SERVO] Servo with ID " << static_cast<int>(id)
              << " is online" << std::endl;
  }
  enable_servos(driver, false);

  driver.set_mode(HIP_LEFT, wobl::real::ServoDriver::POSITION);
  driver.set_mode(HIP_RIGHT, wobl::real::ServoDriver::POSITION);
  driver.set_mode(WHEEL_LEFT, wobl::real::ServoDriver::VELOCITY);
  driver.set_mode(WHEEL_RIGHT, wobl::real::ServoDriver::VELOCITY);

  msg_state.mutable_position()->Resize(4, 0.0);
  msg_state.mutable_velocity()->Resize(4, 0.0);
  msg_state.mutable_effort()->Resize(4, 0.0);

  std::cout << "[SERVO] Servo driver initialized successfully" << std::endl;
  return true;
}

bool has_pending_command(int idx, const wobl::msg::JointCommand &cur,
                         const wobl::msg::JointCommand &prev) {
  if (cur.timestamp() != prev.timestamp() && prev.timestamp() == 0.0)
    return true;

  if (cur.position_size() != 4 && cur.velocity_size() != 4) {
    std::cerr << "[SERVO] Invalid command size" << std::endl;
    return false;
  }

  return cur.position(idx) != prev.position(idx) ||
         cur.velocity(idx) != prev.velocity(idx);
}

int main(int, char **) {
  wobl::Node node;
  wobl::real::ServoDriver driver;

  wobl::msg::JointState msg_state;
  wobl::msg::JointCommand msg_command, msg_command_prev;
  wobl::msg::JointEnable msg_enable;

  double last_state_publish_time = 0.0;

  initialize(driver, msg_state);

  int state_pub = node.add_pub("joint_state");
  node.add_sub("joint_command", &msg_command);
  node.add_sub("joint_enable", &msg_enable);

  node.add_timer(
      [&]() {
        enable_servos(driver, msg_enable.enable());
        if (!is_enabled)
          return;

        for (size_t i = 0; i < servo_ids.size(); ++i) {
          if (!has_pending_command(i, msg_command, msg_command_prev))
            continue;

          u8 id = servo_ids[i];
          float mirror_scalar =
              (id == HIP_RIGHT || id == WHEEL_RIGHT) ? 1.0f : -1.0f;
          if (id == HIP_LEFT || id == HIP_RIGHT) {
            driver.write_position(id, mirror_scalar * msg_command.position(i),
                                  msg_command.velocity(i), 0.2);
          } else if (id == WHEEL_LEFT || id == WHEEL_RIGHT) {
            driver.write_velocity(id, mirror_scalar * msg_command.velocity(i),
                                  0.35);
          }
        }

        double now = node.clock();
        if (now - last_state_publish_time >= 0.02) {
          last_state_publish_time = now;
          msg_state.set_timestamp(now);

          for (size_t i = 0; i < servo_ids.size(); ++i) {
            u8 id = servo_ids[i];
            float mirror_scalar =
                (id == HIP_RIGHT || id == WHEEL_RIGHT) ? 1.0f : -1.0f;
            auto servo_state = driver.read_state(servo_ids[i]);
            msg_state.set_position(i, mirror_scalar * servo_state.position_rad);
            msg_state.set_velocity(i, mirror_scalar * servo_state.velocity_rps);
            msg_state.set_effort(i, servo_state.current_amps);
          }
          node.send(state_pub, msg_state);
        }
      },
      100);

  node.spin();
  enable_servos(driver, false);
  std::cout << "[SERVO] Exiting..." << std::endl;
  return 0;
}