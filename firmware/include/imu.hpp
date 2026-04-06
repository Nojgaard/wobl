#pragma once

#include <ICM_20948.h>

class IMU {
public:
  struct Data {
    float orientation[4];
    float acc[3];
    float gyr[3];
  };

  IMU();
  bool initialize(SPIClass &spi, uint8_t csPin);
  bool try_read(IMU::Data &data);
  bool status() const;
  void print_biases();

  ICM_20948_SPI icm_;

private:
  void load_biases();

  icm_20948_DMP_data_t data_dmp_;
};