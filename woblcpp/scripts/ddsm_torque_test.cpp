#include <atomic>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <wobl/real/ddsm315_driver.hpp>

using namespace wobl::real;
using clock = std::chrono::steady_clock;

constexpr double KI = 0.37; // Motor torque constant (Nm/A)

struct TestConfig {
  std::vector<float> currents_A = {0.0f, 0.1f, 0.2f, 0.3f, -0.2f, 0.0f};
  std::vector<int> durations_s = {2, 3, 3, 3, 3, 2};
};

struct TerminalGuard {
  termios orig_state;
  TerminalGuard() {
    tcgetattr(STDIN_FILENO, &orig_state);
    termios new_state = orig_state;
    new_state.c_lflag &= ~(ICANON | ECHO);
    new_state.c_cc[VMIN] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_state);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
  }
  ~TerminalGuard() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_state); }
};

struct TestMetrics {
  double target_current = 0.0;
  double actual_current = 0.0;
  double velocity_rps = 0.0;
  double estimated_torque = 0.0;
  double current_error = 0.0;
  double timestamp = 0.0;
  double acceleration_rps2 = 0.0; // Add acceleration
  
  void calculate(float target_A, const DDSM315Driver::Feedback& fb, double time_s, double prev_vel = 0.0, double dt = 0.01) {
    target_current = target_A;
    actual_current = fb.current_amp;
    velocity_rps = fb.velocity_rps;
    estimated_torque = KI * actual_current;
    current_error = std::abs(target_current - actual_current);
    timestamp = time_s;
    acceleration_rps2 = (velocity_rps - prev_vel) / dt;
  }
  
  void print() const {
    std::cout << std::fixed << std::setprecision(3)
              << "T:" << std::setw(6) << target_current << "A"
              << " | Act:" << std::setw(6) << actual_current << "A"
              << " | Err:" << std::setw(5) << current_error << "A"
              << " | RPS:" << std::setw(7) << velocity_rps
              << " | τ:" << std::setw(7) << estimated_torque << "Nm"
              << " | α:" << std::setw(7) << acceleration_rps2 << "rps²\n";
  }
};

bool checkEmergencyStop() {
  char c;
  return (read(STDIN_FILENO, &c, 1) > 0 && c == 'q');
}

struct ConstantCalibration {
  double estimated_inertia = 0.0; // kg·m²
  double estimated_kt = 0.0;      // Nm/A
  int samples = 0;
  
  void accumulate(const TestMetrics& m, double prev_vel) {
    // During acceleration phases with minimal load, estimate J and Kt
    // τ_motor = τ_load + J·α
    // If load is small: τ_motor ≈ J·α
    // Kt·I ≈ J·α  →  Kt ≈ J·α/I
    
    if (std::abs(m.actual_current) > 0.05 && std::abs(m.acceleration_rps2) > 0.1) {
      // Skip first phase (settling)
      double angular_accel = m.acceleration_rps2 * 2.0 * M_PI; // rad/s²
      
      // Estimate motor constant from power balance
      // Assuming small friction: Kt ≈ τ/I
      double kt_sample = m.estimated_torque / m.actual_current;
      
      estimated_kt += kt_sample;
      samples++;
    }
  }
  
  void finalize() {
    if (samples > 0) {
      estimated_kt /= samples;
    }
  }
  
  void print() const {
    std::cout << "\n=== Constant Calibration ===\n";
    std::cout << "Estimated Kt: " << std::fixed << std::setprecision(4) 
              << estimated_kt << " Nm/A (assumed: " << KI << " Nm/A)\n";
    std::cout << "Deviation: " << std::fixed << std::setprecision(2)
              << ((estimated_kt / KI - 1.0) * 100.0) << "%\n";
    std::cout << "Calibration samples: " << samples << "\n";
    std::cout << "\nNote: For accurate Kt/J estimation, apply known external load\n";
  }
};

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <motor_id>\n";
    return 1;
  }

  int motor_id = std::atoi(argv[1]);
  if (motor_id < 0 || motor_id > 254) {
    std::cerr << "Invalid motor_id: " << motor_id << "\n";
    return 1;
  }

  TestConfig config;
  if (config.currents_A.size() != config.durations_s.size()) {
    std::cerr << "Configuration error: mismatched array sizes\n";
    return 1;
  }

  // Initialize driver
  std::cout << "Initializing servo driver...\n";
  DDSM315Driver driver;
  if (!driver.is_port_open() || !driver.ping(motor_id)) {
    std::cerr << "Failed to connect to motor " << motor_id << "\n";
    return 1;
  }

  if (!driver.set_mode(motor_id, DDSM315Driver::CURRENT_LOOP)) {
    std::cerr << "Failed to set CURRENT_LOOP mode\n";
    return 1;
  }

  // Setup terminal and timing
  TerminalGuard terminal_guard;
  std::cout << "Press 'q' for emergency stop\n";
  std::cout << "KI constant: " << KI << " Nm/A\n\n";

  const auto write_period = std::chrono::milliseconds(10);
  const auto print_period = std::chrono::milliseconds(100);
  
  auto test_start = clock::now();
  auto last_print = test_start;
  
  long total_duration = 0;
  for (int d : config.durations_s) total_duration += d;

  DDSM315Driver::Feedback fb;
  TestMetrics metrics;
  std::vector<TestMetrics> summary;
  ConstantCalibration calibration;
  
  double prev_velocity = 0.0;
  auto prev_time = test_start;

  // Main test loop
  while (true) {
    if (checkEmergencyStop()) {
      std::cout << "\nEmergency stop!\n";
      driver.set_current(motor_id, 0.0f, fb);
      break;
    }

    auto now = clock::now();
    double dt = std::chrono::duration<double>(now - prev_time).count();
    
    auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - test_start).count();
    if (elapsed_s >= total_duration) break;

    // Find current test phase
    long accum = 0;
    size_t phase = 0;
    for (; phase < config.durations_s.size(); ++phase) {
      if (elapsed_s < accum + config.durations_s[phase]) break;
      accum += config.durations_s[phase];
    }

    float target_A = config.currents_A[phase];
    
    if (!driver.set_current(motor_id, target_A, fb)) {
      std::cerr << "Warning: set_current failed\n";
    }

    // Calculate metrics
    double time_s = std::chrono::duration<double>(now - test_start).count();
    metrics.calculate(target_A, fb, time_s, prev_velocity, dt);

    // Accumulate calibration data
    calibration.accumulate(metrics, prev_velocity);

    // Print status
    if (now - last_print >= print_period) {
      metrics.print();
      last_print = now;
    }
    
    summary.push_back(metrics);
    prev_velocity = fb.velocity_rps;
    prev_time = now;
    std::this_thread::sleep_for(write_period);
  }

  // Print summary statistics
  if (!summary.empty()) {
    double avg_error = 0.0, max_error = 0.0;
    for (const auto& m : summary) {
      avg_error += m.current_error;
      max_error = std::max(max_error, m.current_error);
    }
    avg_error /= summary.size();
    
    std::cout << "\n=== Test Summary ===\n";
    std::cout << "Samples: " << summary.size() << "\n";
    std::cout << "Avg current error: " << avg_error << " A\n";
    std::cout << "Max current error: " << max_error << " A\n";
    std::cout << "KI constant used: " << KI << " Nm/A\n";
    
    calibration.finalize();
    calibration.print();
  }

  return 0;
}