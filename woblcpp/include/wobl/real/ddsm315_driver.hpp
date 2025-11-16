#pragma once
#include <string>
#include <cstdint>

namespace wobl::real {

class DDSM315Driver {
public:
  enum Mode { CURRENT_LOOP = 0x01, VELOCITY_LOOP = 0x02, POSITION_LOOP = 0x03 };

  enum Error {
    SENSOR_ERROR = 0x01,
    OVERCURRENT_ERROR = 0x02,
    PHASE_OVERCURRENT_ERROR = 0x04,
    STALL_ERROR = 0x08,
    TROUBLESHOOTING_ERROR = 0x10
  };

  struct Feedback {
    Mode mode;
    float current_amp;
    float velocity_rps;
    float position_rad;
  };

  DDSM315Driver(std::string port = "/dev/ttyACM0");
  ~DDSM315Driver();

  bool is_port_open() const;
  
  bool ping(int id);
  bool set_mode(int id, Mode mode);
  bool set_id(int new_id);
  bool set_rps(int id, float rps, Feedback &feedback); // -35 ~ 35 rps
  bool get_feedback(int id, Feedback &feedback);

private:
  void open_port(std::string port);
  void close_port();

  bool read_response(Feedback &feedback);

  int port_fd_;

  const static uint8_t PACKET_SIZE = 10;
  uint8_t packet[PACKET_SIZE];
};

} // namespace wobl::real