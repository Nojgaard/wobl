#include <iostream>
#include <wobl/msg/joint.pb.h>
#include <wobl/node.hpp>
#include <wobl/real/ddsm315_driver.hpp>
#include <wobl/real/servo_driver.hpp>

using namespace wobl::real;

enum JointToId {
  SERVO_LEFT = 0,
  SERVO_RIGHT = 5,
  WHEEL_LEFT = 1,
  WHEEL_RIGHT = 2,
};
int index_to_id[] = {SERVO_LEFT, SERVO_RIGHT, WHEEL_LEFT, WHEEL_RIGHT};

wobl::Node node;

wobl::msg::JointCommand msg_command_prev;
wobl::msg::JointCommand msg_command;
wobl::msg::JointState msg_state;

int pub_state = -1;

wobl::real::DDSM315Driver ddsm_driver("/dev/ttyAMA0");
wobl::real::ServoDriver st_driver("/dev/ttyUSB0");

wobl::real::DDSM315Driver::Feedback ddsm_feedback;

void enable_motors(bool enable) {
  for (int id : {SERVO_LEFT, SERVO_RIGHT}) {
    if (!st_driver.enable_torque(id, enable))
      std::cerr << "[MOTOR] Failed to set torque for servo with ID " << id
                << std::endl;
  }
  for (int id : {WHEEL_LEFT, WHEEL_RIGHT}) {
    if (!enable) {
      // To disable torque on DDSM315, set velocity to 0
      if (!ddsm_driver.set_rps(id, 0.0f, ddsm_feedback))
        std::cerr << "[MOTOR] Failed to stop DDSM315 motor with ID " << id
                  << std::endl;
    }
  }
  std::cout << "[MOTOR] Torque " << (enable ? "enabled" : "disabled")
            << " for all motors" << std::endl;
}

void init_msg() {
  msg_state.mutable_position()->Resize(4, 0.0);
  msg_state.mutable_velocity()->Resize(4, 0.0);
  msg_state.mutable_effort()->Resize(4, 0.0);

  msg_command.set_timestamp(0.0);
  msg_command.mutable_position()->Resize(4, 0.0);
  msg_command.mutable_velocity()->Resize(4, 0.0);
  msg_command_prev.CopyFrom(msg_command);
}

bool begin_ddsm_driver() {
  if (!ddsm_driver.is_port_open()) {
    std::cerr << "[MOTOR] Failed to open port for DDSM315 Driver" << std::endl;
    return false;
  }

  for (int id : {WHEEL_LEFT, WHEEL_RIGHT}) {
    if (!ddsm_driver.ping(id)) {
      std::cerr << "[MOTOR] Failed to ping DDSM315 motor with ID " << id
                << std::endl;
      return false;
    }
    std::cout << "[MOTOR] DDSM315 motor with ID " << id << " is online"
              << std::endl;
  }

  std::cout << "[MOTOR] DDSM315 Driver initialized successfully" << std::endl;
  return true;
}

bool begin_st_driver() {
  if (!st_driver.initialize()) {
    std::cerr << "[MOTOR] Failed to initialize Servo Driver" << std::endl;
    return false;
  }

  for (int id : {SERVO_LEFT, SERVO_RIGHT}) {
    if (!st_driver.ping(id)) {
      std::cerr << "[MOTOR] Failed to ping servo with ID " << id << std::endl;
      return false;
    }

    st_driver.set_mode(id, wobl::real::ServoDriver::POSITION);
    auto servo_feedback = st_driver.read_state(id);
    st_driver.write_position(id, servo_feedback.position_rad, 1.0);
    msg_command.set_position(id, servo_feedback.position_rad);
    msg_command.set_velocity(id, 1.0);
    std::cout << "[MOTOR] Servo with ID " << id << " is online" << std::endl;
  }

  std::cout << "[MOTOR] Servo Driver initialized successfully" << std::endl;
  return true;
}

bool has_pending_command(int idx) {
  if (msg_command.timestamp() == msg_command_prev.timestamp())
    return false;
  return msg_command.position(idx) != msg_command_prev.position(idx) ||
         msg_command.velocity(idx) != msg_command_prev.velocity(idx);
}

void actuate() {
  if (msg_command.position_size() != 4 || msg_command.velocity_size() != 4) {
    std::cerr << "[MOTOR] Invalid command message size" << std::endl;
    return;
  }

  for (int i = 0; i < 4; ++i) {
    if (!has_pending_command(i))
      continue;

    int motor_id = index_to_id[i];
    float mirror_scalar =
        (motor_id == SERVO_RIGHT || motor_id == WHEEL_RIGHT) ? 1.0f : -1.0f;
    float pos = msg_command.position(i);
    float vel = msg_command.velocity(i);
    
    if (motor_id == SERVO_LEFT || motor_id == SERVO_RIGHT) {
      st_driver.write_position(motor_id, mirror_scalar * pos, vel, 0.2);
    } else if (motor_id == WHEEL_LEFT || motor_id == WHEEL_RIGHT) {
      ddsm_driver.set_rps(motor_id, mirror_scalar * vel, ddsm_feedback);

      msg_state.set_position(i, mirror_scalar * ddsm_feedback.position_rad);
      msg_state.set_velocity(i, mirror_scalar * ddsm_feedback.velocity_rps);
      msg_state.set_effort(i, ddsm_feedback.current_amp);
    }
  }

  msg_command_prev.CopyFrom(msg_command);
}

void publish_state() {
  double now = node.clock();
  msg_state.set_timestamp(now);

  for (int i = 0; i < 2; ++i) {
    int motor_id = index_to_id[i];
    float mirror_scalar =
        (motor_id == SERVO_RIGHT || motor_id == WHEEL_RIGHT) ? 1.0f : -1.0f;
    auto servo_state = st_driver.read_state(motor_id);
    msg_state.set_position(i, mirror_scalar * servo_state.position_rad);
    msg_state.set_velocity(i, mirror_scalar * servo_state.velocity_rps);
    msg_state.set_effort(i, servo_state.current_amps);
  }

  node.send(pub_state, msg_state);
}

int main() {
  init_msg();
  if (!begin_ddsm_driver() || !begin_st_driver()) {
    return -1;
  }
  enable_motors(true);

  pub_state = node.add_pub("joint_state");
  node.add_sub("joint_command", &msg_command);

  node.add_timer(actuate, 100);
  node.add_timer(publish_state, 50);

  node.spin();

  enable_motors(false);
  std::cout << "[MOTOR] Exiting..." << std::endl;

  return 0;
}