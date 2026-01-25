#include <iostream>
#include <wobl/msg/messages.pb.h>
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

wobl::msg::JointCommand msg_command_prev;
wobl::msg::JointCommand msg_command;
wobl::msg::JointState msg_state;

int pub_state = -1;

wobl::real::DDSM315Driver ddsm_driver("/dev/ttyACM0");
wobl::real::ServoDriver st_driver("/dev/ttyAMA1");

wobl::real::DDSM315Driver::Feedback ddsm_feedback;

//const float WHEEL_KT = 1 / 0.37f; // Nm/A
//const float WHEEL_KT = 1 / 0.15f; // Nm/A
const float WHEEL_KT = 1;

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
    ddsm_driver.set_mode(id, wobl::real::DDSM315Driver::CURRENT_LOOP);
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

  int idx = 0;
  for (int id : {SERVO_LEFT, SERVO_RIGHT}) {
    if (!st_driver.ping(id)) {
      std::cerr << "[MOTOR] Failed to ping servo with ID " << id << std::endl;
      return false;
    }
    st_driver.set_mode(id, wobl::real::ServoDriver::POSITION);
    auto servo_feedback = st_driver.read_state(id);
    st_driver.write_position(idx, servo_feedback.position_rad, 1.0);
    msg_command.set_position(idx, servo_feedback.position_rad);
    msg_command.set_velocity(idx, 1.0);
    std::cout << "[MOTOR] Servo with ID " << id << " is online" << std::endl;
    std::cout << "[MOTOR]   Initial Position: " << servo_feedback.position_rad
              << " rad" << std::endl;
    idx++;
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

void actuate_servos(wobl::Node &node) {
  //auto start_time = std::chrono::steady_clock::now();
  for (int i = 0; i < 2; ++i) {
    if (!has_pending_command(i))
      continue;

    int motor_id = index_to_id[i];
    std::cout << "[MOTOR] Actuating servo ID " << motor_id << std::endl;
    float mirror_scalar = (motor_id == SERVO_RIGHT) ? 1.0f : -1.0f;
    float pos = msg_command.position(i);
    float vel = msg_command.velocity(i);

    st_driver.write_position(motor_id, mirror_scalar * pos, vel, 0.2);
  }
  
  //std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time;
  //std::cout << "[MOTOR] STServo actuation took " << elapsed.count() * 1000.0 << " ms" << std::endl;
}

void actuate_ddsm(wobl::Node &node) {
  auto start_time = std::chrono::steady_clock::now();
  for (int i = 2; i < 4; ++i) {
    int motor_id = index_to_id[i];
    float mirror_scalar = (motor_id == WHEEL_LEFT) ? 1.0f : -1.0f;
    float vel = msg_command.velocity(i);

    bool success = ddsm_driver.set_current(motor_id, mirror_scalar * (vel * WHEEL_KT), ddsm_feedback);
    // bool success = ddsm_driver.set_rps(motor_id, mirror_scalar * vel, ddsm_feedback);
    // Try once more if failed
    if (!success)
      success = ddsm_driver.set_current(motor_id, mirror_scalar * (vel * WHEEL_KT), ddsm_feedback);
    if (!success) {
      std::cerr << "[MOTOR] Warning: Failed to set velocity for DDSM315 motor with ID "
                << motor_id << std::endl;
      continue;
    }

    msg_state.set_position(i, mirror_scalar * ddsm_feedback.position_rad);
    msg_state.set_velocity(i, mirror_scalar * ddsm_feedback.velocity_rps);
    msg_state.set_effort(i, ddsm_feedback.current_amp);
  }

  //std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time;
  //std::cout << "[MOTOR] DDSM315 actuation took " << elapsed.count() * 1000.0 << " ms" << std::endl;
}

void actuate_and_publish_state(wobl::Node &node) {
  if (msg_command.position_size() != 4 || msg_command.velocity_size() != 4) {
    std::cerr << "[MOTOR] Invalid command message size" << std::endl;
    return;
  }

  actuate_servos(node);
  actuate_ddsm(node);

  msg_command_prev.CopyFrom(msg_command);

  double now = node.clock();
  msg_state.set_timestamp(now);
  node.send(pub_state, msg_state);
}

void update_servo_state(wobl::Node &node) {
  for (int i = 0; i < 2; ++i) {
    int motor_id = index_to_id[i];
    float mirror_scalar = (motor_id == SERVO_RIGHT) ? 1.0f : -1.0f;
    auto servo_state = st_driver.read_state(motor_id);
    msg_state.set_position(i, mirror_scalar * servo_state.position_rad);
    msg_state.set_velocity(i, mirror_scalar * servo_state.velocity_rps);
    msg_state.set_effort(i, servo_state.current_amps);
  }
}

int main() {
  wobl::Node node;
  init_msg();
  if (!begin_ddsm_driver() 
   || !begin_st_driver()
  ) {
    return -1;
  }
  enable_motors(true);

  pub_state = node.add_pub("joint_state");
  node.add_sub("joint_command", &msg_command);

  node.add_timer([&node]() { actuate_and_publish_state(node); }, 100);
  //node.add_timer([&node]() { update_servo_state(node); }, 20);

  node.spin();

  enable_motors(false);
  std::cout << "[MOTOR] Exiting..." << std::endl;

  return 0;
}