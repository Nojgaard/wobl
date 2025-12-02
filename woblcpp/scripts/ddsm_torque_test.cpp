#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <wobl/real/ddsm315_driver.hpp>

using namespace wobl::real;

constexpr double KI = 0.37; // Motor torque constant (Nm/A)
constexpr double WHEEL_MASS = 0.35; // kg
constexpr double WHEEL_RADIUS = 0.039; // m

// Calculate wheel moment of inertia (solid disk approximation)
constexpr double WHEEL_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;

struct TerminalGuard {
  termios orig;
  TerminalGuard() {
    tcgetattr(STDIN_FILENO, &orig);
    termios raw = orig;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  }
  ~TerminalGuard() { tcsetattr(STDIN_FILENO, TCSANOW, &orig); }
};

bool check_quit() {
  char c;
  return read(STDIN_FILENO, &c, 1) > 0 && c == 'q';
}

void print_status(double t, float target, const DDSM315Driver::Feedback& fb) {
  double torque = KI * fb.current_amp;
  double error = std::abs(target - fb.current_amp);
  
  std::cout << std::fixed << std::setprecision(3)
            << "t:" << std::setw(5) << t << "s"
            << " | I_tgt:" << std::setw(5) << target << "A"
            << " | I_act:" << std::setw(5) << fb.current_amp << "A"
            << " | err:" << std::setw(5) << error << "A"
            << " | ω:" << std::setw(6) << fb.velocity_rps << "rps"
            << " | τ:" << std::setw(6) << torque << "Nm\n";
}

void calibrate_kt(const std::vector<float>& currents, 
                  const std::vector<DDSM315Driver::Feedback>& feedbacks,
                  const std::vector<double>& times) {
  if (feedbacks.size() < 2) return;
  
  double sum_kt = 0.0;
  int valid_samples = 0;
  
  for (size_t i = 1; i < feedbacks.size(); ++i) {
    double dt = times[i] - times[i-1];
    if (dt <= 0) continue;
    
    double dw = feedbacks[i].velocity_rps - feedbacks[i-1].velocity_rps;
    double alpha = (dw * 2.0 * M_PI) / dt; // angular acceleration (rad/s²)
    
    double avg_current = (feedbacks[i].current_amp + feedbacks[i-1].current_amp) / 2.0;
    
    // During acceleration with current applied
    if (std::abs(avg_current) > 0.05 && std::abs(alpha) > 1.0) {
      // τ_motor = J_wheel * α  (neglecting friction during transients)
      // Kt * I = J_wheel * α
      // Kt = (J_wheel * α) / I
      
      double kt_sample = (WHEEL_INERTIA * alpha) / avg_current;
      
      // Filter outliers
      if (kt_sample > 0.1 && kt_sample < 1.0) {
        sum_kt += kt_sample;
        valid_samples++;
      }
    }
  }
  
  if (valid_samples > 0) {
    double estimated_kt = sum_kt / valid_samples;
    std::cout << "\n=== Kt Calibration ===\n";
    std::cout << "Wheel inertia: " << WHEEL_INERTIA << " kg·m²\n";
    std::cout << "Estimated Kt: " << std::fixed << std::setprecision(4) 
              << estimated_kt << " Nm/A\n";
    std::cout << "Assumed Kt: " << KI << " Nm/A\n";
    std::cout << "Deviation: " << std::fixed << std::setprecision(1)
              << ((estimated_kt / KI - 1.0) * 100.0) << "%\n";
    std::cout << "Valid samples: " << valid_samples << "\n";
  } else {
    std::cout << "Insufficient data for Kt calibration\n";
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <motor_id>\n";
    return 1;
  }

  int motor_id = std::atoi(argv[1]);
  if (motor_id < 0 || motor_id > 254) {
    std::cerr << "Invalid motor_id\n";
    return 1;
  }

  // Calibration sequence - optimized for Kt estimation
  const std::vector<float> currents = {
    0.0f,   // 2s - settle/zero
    0.3f,   // 3s - strong acceleration 
    0.0f,   // 2s - coast down (measure friction)
    0.5f,   // 3s - maximum safe acceleration
    0.0f,   // 2s - coast down
    -0.3f,  // 3s - reverse acceleration
    0.0f    // 2s - final settle
  };
  const std::vector<int> durations = {2, 3, 2, 3, 2, 3, 2};

  // Initialize
  std::cout << "Initializing motor " << motor_id << "...\n";
  DDSM315Driver driver;
  if (!driver.is_port_open() || !driver.ping(motor_id)) {
    std::cerr << "Failed to connect\n";
    return 1;
  }

  if (!driver.set_mode(motor_id, DDSM315Driver::CURRENT_LOOP)) {
    std::cerr << "Failed to set CURRENT_LOOP mode\n";
    return 1;
  }

  TerminalGuard term;
  std::cout << "Press 'q' to stop. KI = " << KI << " Nm/A\n\n";

  const auto dt_control = std::chrono::milliseconds(10);
  const auto dt_print = std::chrono::milliseconds(100);
  
  auto t_start = std::chrono::steady_clock::now();
  auto t_last_print = t_start;

  DDSM315Driver::Feedback fb;
  double sum_error = 0.0;
  int samples = 0;

  std::vector<DDSM315Driver::Feedback> feedback_log;
  std::vector<float> current_log;
  std::vector<double> time_log;

  // Main loop
  while (true) {
    if (check_quit()) {
      std::cout << "\nStopping...\n";
      driver.set_current(motor_id, 0.0f, fb);
      break;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - t_start).count();

    // Find current phase
    int total = 0;
    size_t phase = 0;
    for (; phase < durations.size(); ++phase) {
      if (elapsed < total + durations[phase]) break;
      total += durations[phase];
    }
    if (phase >= durations.size()) break;

    float target = currents[phase];
    driver.set_current(motor_id, target, fb);

    // Log for calibration
    double t = std::chrono::duration<double>(now - t_start).count();
    feedback_log.push_back(fb);
    current_log.push_back(target);
    time_log.push_back(t);

    // Statistics
    sum_error += std::abs(target - fb.current_amp);
    samples++;

    // Print periodically
    if (now - t_last_print >= dt_print) {
      double t = std::chrono::duration<double>(now - t_start).count();
      print_status(t, target, fb);
      t_last_print = now;
    }

    std::this_thread::sleep_for(dt_control);
  }

  // Summary
  if (samples > 0) {
    std::cout << "\nAvg error: " << (sum_error / samples) << " A over " 
              << samples << " samples\n";
    
    calibrate_kt(current_log, feedback_log, time_log);
  }

  return 0;
}