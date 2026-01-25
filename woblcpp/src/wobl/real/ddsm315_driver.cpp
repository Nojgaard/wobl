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
  tty.c_cc[VTIME] = 0; // Read timeout
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

ssize_t read_data(int fd, uint8_t* buffer, size_t size) {
  size_t total_read = 0;
  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(10);
  const auto first_byte_timeout = std::chrono::milliseconds(4);
  
  // Read until we get all expected bytes or timeout
  while (total_read < size) {
    ssize_t bytes_read = read(fd, buffer + total_read, size - total_read);
    
    if (bytes_read > 0) {
      total_read += bytes_read;
    } else if (bytes_read < 0) {
      std::cerr << "Read error: " << errno << std::endl;
      return total_read;
    }
    
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (total_read != size && elapsed > timeout) {
      //std::cerr << "Read timeout: expected " << (int)size
      //          << " bytes, got " << total_read << " bytes." << std::endl;
      return total_read;
    }

    if (total_read == 0 && elapsed > first_byte_timeout) {
      // If no bytes have been read yet, use a shorter timeout for the first byte
      //std::cerr << "First byte read timeout." << std::endl;
      return total_read;
    }
  }
  return total_read;
}

bool DDSM315Driver::read_response(Feedback &feedback) {
  int expected_id = packet[0];
  ssize_t bytes_read = read_data(port_fd_, packet, PACKET_SIZE);
  if (bytes_read != PACKET_SIZE) {
    //std::cerr << "Read error: expected " << (int)PACKET_SIZE
    //          << " bytes, got " << bytes_read << " bytes." << std::endl;
    return false;
  }

  uint8_t crc = maxim_crc8(packet, PACKET_SIZE - 1);
  if (crc != packet[PACKET_SIZE - 1]) {
    std::cerr << "CRC error: expected " << (int)packet[PACKET_SIZE - 1]
              << ", got " << (int)crc << std::endl;
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

  if (packet[0] != expected_id) {
    std::cerr << "Unexpected motor ID\n";
    return false;
  }
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

bool DDSM315Driver::set_current(int id, float current_amp, Feedback &feedback) {
  if (!is_port_open()) {
    return false;
  }
  tcflush(port_fd_, TCIOFLUSH);
  uint8_t acceleration = 0;

  current_amp = std::clamp(current_amp, -0.8f, 0.8f);
  float current2lsb = 32767.0f / 8.0f; // 4095.875 LSB/A
  int16_t current_lsb = static_cast<int16_t>(std::round(current_amp * current2lsb));

  for (int i = 0; i < PACKET_SIZE; ++i)
    packet[i] = 0;

  packet[0] = id;
  packet[1] = 0x64; // Command for setting RPS
  packet[2] = (current_lsb >> 8) & 0xFF;
  packet[3] = current_lsb & 0xFF;
  packet[6] = acceleration;
  packet[9] = maxim_crc8(packet, PACKET_SIZE - 1);

  ssize_t bytes_written = write(port_fd_, packet, PACKET_SIZE);
  if (bytes_written != PACKET_SIZE){
    std::cerr << "Write error: expected " << (int)PACKET_SIZE
              << " bytes, wrote " << bytes_written << " bytes." << std::endl;
    return false;
  }

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