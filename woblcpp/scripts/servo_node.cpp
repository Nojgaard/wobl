#include <iostream>
#include <vector>
#include <wobl/msg/messages.pb.h>
#include <wobl/node.hpp>
#include <wobl/real/servo_driver.hpp>

enum JointToId {
  HIP_LEFT = 0,
  HIP_RIGHT = 5,
  WHEEL_LEFT = 6,
  WHEEL_RIGHT = 2,
};

// 1:2 gear ratio for the wheel to increase speed at the cost of torque.
const float WHEEL_GEAR_RATIO = 0.5;

const std::vector<uint8_t> servo_ids = {HIP_LEFT, HIP_RIGHT, WHEEL_LEFT, WHEEL_RIGHT};
const std::vector<float> world2servo_coords = {-1.0f, 1.0f, -WHEEL_GEAR_RATIO, WHEEL_GEAR_RATIO};
const std::vector<float> servo2world_coords = {-1.0f, 1.0f, -1.0f / WHEEL_GEAR_RATIO, 1.0f / WHEEL_GEAR_RATIO};
bool is_enabled = false;

void enable_servos(wobl::real::ServoDriver &driver, bool enable, bool force = false) {
  if (enable == is_enabled && !force)
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
  enable_servos(driver, false, true);

  driver.set_mode(HIP_LEFT, wobl::real::ServoDriver::POSITION);
  driver.set_mode(HIP_RIGHT, wobl::real::ServoDriver::POSITION);
  driver.set_mode(WHEEL_LEFT, wobl::real::ServoDriver::VELOCITY);
  driver.set_mode(WHEEL_RIGHT, wobl::real::ServoDriver::VELOCITY);
  auto left_servo_state = driver.read_state(HIP_LEFT);
  auto right_servo_state = driver.read_state(HIP_RIGHT);
  driver.write_position(HIP_LEFT, left_servo_state.position_rad, 1.0);
  driver.write_position(HIP_RIGHT, right_servo_state.position_rad, 1.0);
  driver.write_velocity(WHEEL_LEFT, 0.0);
  driver.write_velocity(WHEEL_RIGHT, 0.0);

  msg_state.mutable_position()->Resize(4, 0.0);
  msg_state.mutable_velocity()->Resize(4, 0.0);
  msg_state.mutable_effort()->Resize(4, 0.0);

  std::cout << "[SERVO] Servo driver initialized successfully" << std::endl;
  return true;
}

bool has_pending_command(int idx, const wobl::msg::JointCommand &cur,
                         const wobl::msg::JointCommand &prev) {
  if (cur.timestamp() == 0.0)
    return false;

  if (prev.timestamp() == 0.0)
    return true;

  if (cur.position().size() <= idx || cur.velocity().size() <= idx ||
      prev.position().size() <= idx || prev.velocity().size() <= idx) {
    std::cerr << "[SERVO] Array size mismatch or invalid index. cur pos:"
              << cur.position().size() << " vel:" << cur.velocity().size()
              << " prev pos:" << prev.position().size()
              << " vel:" << prev.velocity().size() << " idx:" << idx
              << std::endl;
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

  if (!initialize(driver, msg_state)) {
    enable_servos(driver, false, true);
    return -1;
  }

  int state_pub = node.add_pub("joint_state");
  node.add_sub("joint_command", &msg_command);
  node.add_sub("joint_enable", &msg_enable);

  enable_servos(driver, true, true);

  node.add_timer(
      [&]() {
        //enable_servos(driver, msg_enable.enable());
        if (!is_enabled)
          return;
        
        for (int i = 0; i < servo_ids.size(); ++i) {
          if (!has_pending_command(i, msg_command, msg_command_prev))
            continue;

          u8 id = servo_ids[i];
          float world2servo = world2servo_coords[i];
          if (id == HIP_LEFT || id == HIP_RIGHT) {
            driver.write_position(id, world2servo * msg_command.position(i),
                                  msg_command.velocity(i), 0.2);
          } else if (id == WHEEL_LEFT || id == WHEEL_RIGHT) {
            driver.write_velocity(id, world2servo * msg_command.velocity(i),
                                  0.35);
          }
        }
        msg_command_prev.CopyFrom(msg_command);

        double now = node.clock();
        if (now - last_state_publish_time >= 0.02) {
          last_state_publish_time = now;
          msg_state.set_timestamp(now);

          for (size_t i = 0; i < servo_ids.size(); ++i) {
            u8 id = servo_ids[i];
            float servo2world = servo2world_coords[i];
            auto servo_state = driver.read_state(servo_ids[i]);
            msg_state.set_position(i, servo2world * servo_state.position_rad);
            msg_state.set_velocity(i, servo2world * servo_state.velocity_rps);
            msg_state.set_effort(i, servo_state.current_amps);
          }
          node.send(state_pub, msg_state);
        }
      },
      80);

  node.spin();
  enable_servos(driver, false, true);
  std::cout << "[SERVO] Exiting..." << std::endl;
  return 0;
}