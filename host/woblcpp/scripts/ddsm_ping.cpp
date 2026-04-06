#include <wobl/real/ddsm315_driver.hpp>
#include <iostream>

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
  std::cout << "OK\n\n";
  return 0;
}