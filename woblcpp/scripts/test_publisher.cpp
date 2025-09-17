#include <iostream>
#include <wobl/msg/imu.pb.h>
#include <wobl/node.hpp>

int main(int, char **) {
  wobl::Node node;

  wobl::msg::Imu msg;
  int pub = node.add_pub("imu");

  node.add_timer(
      [&]() {
        std::cout << "Sending message" << std::endl;
        node.send(pub, msg);
      },
      1);
  node.spin();
  return 0;
}