#pragma once
#define ICM_20948_USE_DMP

#include <ICM_20948.h>

class IMU {
public:
  IMU();
  bool initialize();
  bool try_read();

  bool status() const;

    ICM_20948_I2C icm_;
private:
  void try_load_bias();

  icm_20948_DMP_data_t data_dmp_;
};