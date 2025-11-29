#include <algorithm>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <wobl/real/ddsm315_driver.hpp>
#include <thread>
#include <chrono>

namespace wobl::real {

uint8_t maxim_crc8(uint8_t *data, unsigned int size) {
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i) {
    uint8_t inbyte = data[i];
    for (unsigned char j = 0; j < 8; ++j) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

DDSM315Driver::DDSM315Driver(std::string port) : port_fd_(-1) {
    open_port(port);
}

DDSM315Driver::~DDSM315Driver() { close_port(); }

void DDSM315Driver::open_port(std::string port) {
  struct termios tty;
  port_fd_ = open(port.c_str(), O_RDWR);

  if (port_fd_ <= 0) {
    std::cerr << "Unable to open port " << port << std::endl;
    close_port();
    return;
  }

  if (tcgetattr(port_fd_, &tty) != 0) {
    std::cerr << "Unable to get attributes for port " << port << std::endl;
    close_port();
    return;
  }

  tty.c_cflag &= ~PARENB;                 // No parity
  tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
  tty.c_cflag &= ~CSIZE;                  // Clear byte size bits
  tty.c_cflag |= CS8;                     // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;                // Disable CTS/RTS
  tty.c_lflag = 0;                        // Make tty raw
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  // Disable any special handling of received bytes
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
  // Prevent conversion of newline to carriage return/line feed
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 1; // Read timeout
  tty.c_cc[VMIN] = 0;

  // Set baud to 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  if (tcsetattr(port_fd_, TCSANOW, &tty) != 0) {
    std::cerr << "Error " << errno << " setting port attributes" << std::endl;
    close_port();
    return;
  }
}

void DDSM315Driver::close_port() {
  if (is_port_open()) {
    close(port_fd_);
    port_fd_ = -1;
  }
}

bool DDSM315Driver::is_port_open() const { return port_fd_ > 0; }

bool DDSM315Driver::ping(int id) {
  Feedback feedback;
  return get_feedback(id, feedback);
}

bool DDSM315Driver::set_mode(int id, Mode mode) {
  if (!is_port_open()) {
    return false;
  }

  for (int i = 0; i < PACKET_SIZE; ++i)
    packet[i] = 0;

  packet[0] = id;
  packet[1] = 0xA0; // Command for setting mode
  packet[9] = mode;

  ssize_t bytes_written = write(port_fd_, packet, PACKET_SIZE);
  return bytes_written == PACKET_SIZE;
}

bool DDSM315Driver::read_response(Feedback &feedback) {
  ssize_t bytes_read = read(port_fd_, packet, PACKET_SIZE);
  if (bytes_read != PACKET_SIZE) {
    return false;
  }

  uint8_t crc = maxim_crc8(packet, PACKET_SIZE - 1);
  if (crc != packet[PACKET_SIZE - 1]) {
    return false;
  }

  uint8_t mode = packet[1];
  int16_t current_lsb = (int16_t)((packet[2] << 8) | packet[3]);
  int16_t velocity_lsb = (int16_t)((packet[4] << 8) | packet[5]);
  uint16_t position_lsb = (packet[6] << 8) | packet[7];
  uint8_t error = packet[8];

  if (error != 0) {
    std::cerr << "Motor error: ";
    if (error & SENSOR_ERROR)
      std::cerr << "SENSOR_ERROR ";
    if (error & OVERCURRENT_ERROR)
      std::cerr << "OVERCURRENT_ERROR ";
    if (error & PHASE_OVERCURRENT_ERROR)
      std::cerr << "PHASE_OVERCURRENT_ERROR ";
    if (error & STALL_ERROR)
      std::cerr << "STALL_ERROR ";
    if (error & TROUBLESHOOTING_ERROR)
      std::cerr << "TROUBLESHOOTING_ERROR ";
    std::cerr << std::endl;
    return false;
  }

  feedback.mode = static_cast<Mode>(mode);

  //-32767~32767 corresponds to -8A~8A (non-motor current range), the data type
  // is signed 16-bit
  feedback.current_amp = (8.0 / 32767.0) * (float)current_lsb;

  // -330~330, unit rpm, the data type is signed 16-bit
  feedback.velocity_rps = (2 * M_PI / 60.0) * (float)velocity_lsb;

  // 0~32767 corresponds to 0 °~360 °, the data type is unsigned 16-bit;
  feedback.position_rad =
      (360.0 / 32767.0) * (M_PI / 180.0) * (float)position_lsb;

  return true;
}

bool DDSM315Driver::set_rps(int id, float rps, Feedback &feedback) {
  if (!is_port_open()) {
    return false;
  }

  float rps2rpm = 60.0f / (2.0f * M_PI);
  int16_t rpm = static_cast<int16_t>(std::round(rps * rps2rpm));
  rpm = std::clamp(rpm, (int16_t)-330, (int16_t)330);
  // The acceleration time per 1rpm is 0.1ms. 
  // When set to 1, the acceleration time per 1rpm is 0.1ms. 
  // When set to 10, the acceleration time per 1rpm is 10*0.1ms=1ms. 
  // When set to 0, the default value is 1, and the acceleration time per 1rpm is 0.1ms.
  uint8_t acceleration = 0;

  for (int i = 0; i < PACKET_SIZE; ++i)
    packet[i] = 0;

  packet[0] = id;
  packet[1] = 0x64; // Command for setting RPS
  packet[2] = (rpm >> 8) & 0xFF;
  packet[3] = rpm & 0xFF;
  packet[6] = acceleration;
  packet[9] = maxim_crc8(packet, PACKET_SIZE - 1);

  ssize_t bytes_written = write(port_fd_, packet, PACKET_SIZE);
  if (bytes_written != PACKET_SIZE)
    return false;

  return read_response(feedback);
}

bool DDSM315Driver::get_feedback(int id, Feedback &feedback) {
  return set_rps(id, 0.0f, feedback);
}

bool DDSM315Driver::set_id(int new_id) {
  if (!is_port_open()) {
    return false;
  }
  if (new_id < 1 || new_id > 253) {
    std::cerr << "New ID must be between 1 and 253" << std::endl;
    return false;
  }

  for (int i = 0; i < PACKET_SIZE; ++i)
    packet[i] = 0;

  packet[0] = 0xAA;
  packet[1] = 0x55;
  packet[2] = 0x53;
  packet[3] = new_id;
  packet[9] = maxim_crc8(packet, PACKET_SIZE - 1);

  // id is only set after 5 successful writes
  for (int i = 0; i < 5; i++) {
    ssize_t bytes_written = write(port_fd_, packet, PACKET_SIZE);
    if (bytes_written != PACKET_SIZE)
      return false;
    usleep(1000); // wait 1ms before resending
  }

  return ping(new_id);
}

} // namespace wobl::real