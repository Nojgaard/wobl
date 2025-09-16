#include <iostream>
#include <wobl/msg/imu.pb.h>
#include <wobl/node.hpp>
#include <wobl/real/imu_driver.hpp>

int main(int, char **) {
  wobl::Node node;
  wobl::real::ImuDriver driver;

  std::cout << "[IMU] Initializing..." << std::endl;
  if (!driver.initialize()) {
    std::cerr << "[IMU] Failed to initialize IMU driver" << std::endl;
    return -1;
  }

  wobl::msg::Imu imu_msg;
  int imu_pub = node.add_pub("imu");

  node.add_timer(
      [&]() {
        bool has_data = driver.try_read(imu_msg);
        imu_msg.set_timestamp(node.clock());

        if (has_data)
          node.send(imu_pub, imu_msg);
      },
      100);
  node.spin();
  return 0;
}