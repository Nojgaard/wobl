#include <iostream>
#include <wobl/msg/joint.pb.h>
#include <wobl/node.hpp>
#include <random>

int main(int, char **) {
  wobl::Node node;

  wobl::msg::JointCommand msg;
  msg.mutable_position()->Resize(4, 0.0);
  msg.mutable_velocity()->Resize(4, 0.0);
  int pub = node.add_pub("test");
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister generator

    // Define the range [min, max)
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  node.add_timer(
      [&]() {
        float rnd = dist(gen);
        for (int i = 0; i < msg.position_size(); i++) {
          msg.set_position(i, rnd);
          msg.set_velocity(i, rnd);
        }
        node.send(pub, msg);
      },
      80);
  node.spin();
  return 0;
}