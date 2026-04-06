#include <iostream>
#include <wobl/real/servo_driver.hpp>

using namespace wobl::real;

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "must include src id and target id as cmd line arguments";
    return 1;
  }

  int src_id = std::atoi(argv[1]);
  int tar_id = std::atoi(argv[2]);

  if (src_id < 0 || src_id > 254 || tar_id < 0 || tar_id > 254) {
    std::cout << "src_id: " << src_id << " and tar_id: " << tar_id << " are invalid\n";
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

  std::cout << "Changing servo id from " << src_id << " to " << tar_id << "... ";
  if (!driver.set_id(src_id, tar_id)) {
    std::cerr << "ERROR\n";
    return -3;
  }
  std::cout << "OK\n\n";
  return 0;
}