#pragma once

#include <ICM_20948.h>

class IMU {
public:
  struct Data {
    float orientation[4];
    float acc[3];
    float gyr[3];
  };

  struct Calibration {
    int32_t gyro[3]{};
    int32_t accel[3]{};
    int32_t mag[3]{};
  };

  IMU();
  bool initialize(SPIClass &spi, uint8_t csPin);
  bool try_read(IMU::Data &data);
  bool status() const;
  IMU::Calibration load_biases();  // Load from NVS (or factory defaults), configure DMP, return loaded state
  IMU::Calibration save_biases();  // Read DMP state, persist to NVS, return what was saved
  void print_biases();             // Debug: print current DMP state

  ICM_20948_SPI icm_;

  icm_20948_DMP_data_t data_dmp_;

private:
  void set_dmp_biases(const IMU::Calibration &cal);  // Internal: write calibration values to DMP
  IMU::Calibration get_dmp_biases();                 // Internal: read calibration values from DMP
};