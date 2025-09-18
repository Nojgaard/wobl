#include <iostream>
#include <wobl/msg/joint.pb.h>
#include <wobl/node.hpp>

int main(int, char **) {
  wobl::Node node;

  wobl::msg::JointCommand msg;
  node.add_sub("test", &msg);

  node.add_timer(
      [&]() {
        std::cout << "Rec message" << std::endl;
        for (int i = 0; i < msg.velocity_size(); i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          std::cout << "Velocity[" << i << "] = " << msg.velocity(i) << std::endl;
        }
      },
      80);
  node.spin();
  return 0;
}