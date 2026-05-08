#include <imu.hpp>
#include "debug.hpp"

// IMU bias values from calibration
const int32_t BIAS_LINEAR_ACCELERATION[3] = {-330752, -826368, 585728};
const int32_t BIAS_ANGULAR_VELOCITY[3] = {-17440, 1344, 12768};
const int32_t BIAS_COMPASS[3] = {-230400, 2479070, -3262980};

IMU::IMU() {}

bool IMU::initialize(SPIClass &spi, uint8_t csPin) {
  bool success = true;

  success &= (icm_.begin(csPin, spi) == ICM_20948_Stat_Ok);
  delay(50);

  auto check = [&](ICM_20948_Status_e ret, const char *name) {
    if (ret != ICM_20948_Stat_Ok) {
      DPRINT(name);
      DPRINT(" failed: ");
      DPRINTLN(icm_.statusString(ret));
      success = false;
    }
  };

  check(icm_.initializeDMP(), "initializeDMP");

  // The weak initializeDMP override in this file may not be invoked by the
  // ESP32 linker. Patch the four rate-dependent DMP constants here explicitly
  // to guarantee 225Hz (divider=4: gyro ~220Hz, accel ~225Hz).
  // Values from InvenSense Application Note (see initializeDMP comments).
  ICM_20948_smplrt_t smplrt;
  smplrt.g = 4;
  smplrt.a = 4;
  check(icm_.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), smplrt), "setSampleRate");
  check(icm_.setGyroSF(4, 3), "setGyroSF"); // divider=4, 2000dps
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]), "ACCEL_ONLY_GAIN");
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]), "ACCEL_ALPHA_VAR");
  const unsigned char accelAVar[4]     = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]), "ACCEL_A_VAR");

  // Use Quat6 (6-axis game rotation vector, no magnetometer). Yaw drift is
  // acceptable for a self-balancing robot, and Quat6 is not rate-limited by
  // the magnetometer ODR (68.75Hz) that caps Quat9 at ~56Hz.
  check(icm_.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR), "enableDMPSensor");
  check(icm_.setDMPODRrate(DMP_ODR_Reg_Quat6, 0), "setDMPODRrate");
  check(icm_.enableFIFO(), "enableFIFO");
  check(icm_.enableDMP(), "enableDMP");
  check(icm_.resetDMP(), "resetDMP");
  check(icm_.resetFIFO(), "resetFIFO");

  return success;
}

bool IMU::try_read(IMU::Data &out_data) {
  // gpm4 and dps2000 are hardcoded in initializeDMP
  constexpr float quat9_scale = 1.0f / 1073741824.0f; // 1 / 2^30
  constexpr float accel_scale = 9.80665f / 8192.0f;   // gpm4: m/s² per LSB
  constexpr float gyro_scale =
      (M_PI / 180.0f) / 2048.0f; // dps2000: rad/s per LSB

  bool got_data = false;
  icm_.readDMPdataFromFIFO(&data_dmp_);

  while ((icm_.status == ICM_20948_Stat_Ok) ||
         (icm_.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if (data_dmp_.header & DMP_header_bitmap_Quat6) {
      float q1 = data_dmp_.Quat6.Data.Q1 * quat9_scale;
      float q2 = data_dmp_.Quat6.Data.Q2 * quat9_scale;
      float q3 = data_dmp_.Quat6.Data.Q3 * quat9_scale;
      float q0_sq = 1.0f - (q1 * q1 + q2 * q2 + q3 * q3);
      float q0 = q0_sq > 0.0f ? std::sqrt(q0_sq)
                              : 0.0f; // guard against drift-induced NaN

      out_data.orientation[0] = q1;
      out_data.orientation[1] = q2;
      out_data.orientation[2] = q3;
      out_data.orientation[3] = q0;
      got_data = true;
    }
    if (data_dmp_.header & DMP_header_bitmap_Accel) {
      const auto &accel = data_dmp_.Raw_Accel.Data;
      out_data.acc[0] = accel.X * accel_scale;
      out_data.acc[1] = accel.Y * accel_scale;
      out_data.acc[2] = accel.Z * accel_scale;
    }
    if (data_dmp_.header & DMP_header_bitmap_Gyro_Calibr) {
      const auto &gyro = data_dmp_.Raw_Gyro.Data;
      out_data.gyr[0] = gyro.X * gyro_scale;
      out_data.gyr[1] = gyro.Y * gyro_scale;
      out_data.gyr[2] = gyro.Z * gyro_scale;
    }
    icm_.readDMPdataFromFIFO(&data_dmp_);
  }

  return got_data;
}

void IMU::load_biases() {
  icm_.setBiasAccelX(BIAS_LINEAR_ACCELERATION[0]);
  icm_.setBiasAccelY(BIAS_LINEAR_ACCELERATION[1]);
  icm_.setBiasAccelZ(BIAS_LINEAR_ACCELERATION[2]);

  icm_.setBiasGyroX(BIAS_ANGULAR_VELOCITY[0]);
  icm_.setBiasGyroY(BIAS_ANGULAR_VELOCITY[1]);
  icm_.setBiasGyroZ(BIAS_ANGULAR_VELOCITY[2]);

  icm_.setBiasCPassX(BIAS_COMPASS[0]);
  icm_.setBiasCPassY(BIAS_COMPASS[1]);
  icm_.setBiasCPassZ(BIAS_COMPASS[2]);
}

void IMU::print_biases() {
  int32_t acc_x, acc_y, acc_z;
  int32_t gyr_x, gyr_y, gyr_z;
  int32_t mag_x, mag_y, mag_z;
  icm_.getBiasAccelX(&acc_x);
  icm_.getBiasAccelY(&acc_y);
  icm_.getBiasAccelZ(&acc_z);

  icm_.getBiasGyroX(&gyr_x);
  icm_.getBiasGyroY(&gyr_y);
  icm_.getBiasGyroZ(&gyr_z);

  icm_.getBiasCPassX(&mag_x);
  icm_.getBiasCPassY(&mag_y);
  icm_.getBiasCPassZ(&mag_z);

  DPRINTLN("Loaded Biases:");
  DPRINTF("  Linear Acceleration: [%d, %d, %d]\n", acc_x, acc_y, acc_z);
  DPRINTF("  Angular Velocity:    [%d, %d, %d]\n", gyr_x, gyr_y, gyr_z);
  DPRINTF("  Compass:             [%d, %d, %d]\n", mag_x, mag_y, mag_z);
}