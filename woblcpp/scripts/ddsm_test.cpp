#include <wobl/real/ddsm315_driver.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace wobl::real;

// Set up non-blocking keyboard input
void setupNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO);
    ttystate.c_cc[VMIN] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Must specify id as argument.";
    return 1;
  }

  int src_id = std::atoi(argv[1]);

  if (src_id < 0 || src_id > 254) {
    std::cout << "src_id: " << src_id << " are invalid\n";
    return 1;
  }

  std::cout << "Initializing servo driver...\n";
  DDSM315Driver driver;
  if (!driver.is_port_open()) {
    std::cerr << "Failed to initialize driver\n";
    return -1;
  }
  std::cout << "Pinging servo " << src_id << "... ";
  if (!driver.ping(src_id)) {
    std::cerr << "no response\n";
    return -2;
  }

  // Set up non-blocking keyboard input for emergency stop
  setupNonBlockingInput();
  std::cout << "Press 'q' at any time for emergency stop\n";

  // Define a sequence of target RPS and durations (seconds)
  // These can be adjusted as needed. Example: ramp, hold, reverse.
  std::vector<float> targets_rps = {0.0f, .2f, 0.5f, -0.5f, 0.0f};
  std::vector<int> durations_s = {1, 2, 2, 1, 1};

  if (targets_rps.size() != durations_s.size()) {
    std::cerr << "targets_rps and durations_s must have same length\n";
    return -3;
  }

    // Timing parameters
  const std::chrono::milliseconds write_period_ms(5); // 100 Hz
  const std::chrono::milliseconds print_period_ms(100); // 10 Hz

  DDSM315Driver::Feedback fb;
  using clock = std::chrono::steady_clock;
  auto seq_start = clock::now();
  long total_sequence_seconds = 0;
  for (int d : durations_s) total_sequence_seconds += d;

  int writes_count = 0;
  auto last_print_time = clock::now();
  int last_print_writes = 0;

  bool emergency_stop = false;
  while (true) {
    // Check for emergency stop
    char c;
    if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
      std::cout << "\nEmergency stop requested!\n";
      emergency_stop = true;
      // Set motor to 0 RPS immediately
      if (!driver.set_rps(src_id, 0.0f, fb)) {
        std::cerr << "Warning: Emergency stop set_rps failed!\n";
      }
      break;
    }

    auto now = clock::now();
    auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - seq_start).count();

    if (elapsed_s >= total_sequence_seconds) break; // done

    // Determine current target index
    long accum = 0;
    size_t cur = 0;
    for (; cur < durations_s.size(); ++cur) {
      accum += durations_s[cur];
      if (elapsed_s < accum) break;
    }

    if (cur >= durations_s.size()) break;

    float target = targets_rps[cur];

    // Call set_rps on main thread; it writes and returns feedback
    if (!driver.set_rps(src_id, target, fb)) {
      std::cerr << "Warning: set_rps failed for id " << src_id << " target " << target << "\n";
      // continue nevertheless
    }
    ++writes_count;

    // Print at 10Hz: check if enough time passed since last print
    auto since_last_print = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time);
    if (since_last_print >= print_period_ms) {
      double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_print_time).count();
      int delta_writes = writes_count - last_print_writes;
      double writes_per_sec = dt > 0.0 ? (double)delta_writes / dt : 0.0;

      std::cout << std::fixed << std::setprecision(3)
                << "FB rps: " << fb.velocity_rps
                << " | writes/s: " << writes_per_sec
                << " | mode: " << fb.mode
                << "\n";

      last_print_time = now;
      last_print_writes = writes_count;
    }

    std::this_thread::sleep_for(write_period_ms);
  }

  if (emergency_stop) {
    std::cout << "Emergency stop completed.\n";
  } else {
    std::cout << "Sequence complete.\n";
  }

  // Restore terminal settings before exit
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);
  ttystate.c_lflag |= ICANON | ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
  
  return 0;
}
