#include <iostream>
#include <wobl/real/servo_driver.hpp>

using namespace wobl::real;

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

  ServoDriver driver;
  std::cout << "Initializing servo driver...\n";
  if (!driver.initialize()) {
    std::cerr << "Failed to initialize driver\n";
    return -1;
  }

  std::cout << "Pinging servo " << src_id << "... ";
  if (!driver.ping(src_id)) {
    std::cerr << "no response\n";
    return -2;
  }
  std::cout << "OK\n\n";

  std::cout << "Setting midpoint for " << src_id << "... ";
  if (!driver.set_midpoint(src_id)) {
    std::cerr << "ERROR\n";
    return -3;
  }
  std::cout << "OK\n\n";
  return 0;
}