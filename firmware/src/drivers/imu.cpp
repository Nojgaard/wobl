#include "debug.hpp"
#include <drivers/imu.hpp>

#include <Preferences.h>

// NVS namespace — owned by the IMU module, not visible to comm_task.
static constexpr const char *kNvsNamespace = "wobl_calib";
static constexpr const char *kNvsKeyGyro = "gyro";
static constexpr const char *kNvsKeyAccel = "accel";
static constexpr const char *kNvsKeyMag = "mag";

// Factory defaults applied when NVS is empty (first boot after flash).
static const int32_t kDefaultBiasAccel[3] = {-330752, -826368, 585728};
static const int32_t kDefaultBiasGyro[3] = {-17440, 1344, 12768};
static const int32_t kDefaultBiasMag[3] = {-230400, 2479070, -3262980};

IMU::IMU() {}

bool IMU::initialize(SPIClass &spi, uint8_t csPin) {
  bool success = true;

  success &= (icm_.begin(csPin, spi, 1000000UL) == ICM_20948_Stat_Ok);
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
  check(icm_.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                           smplrt),
        "setSampleRate");
  check(icm_.setGyroSF(4, 3), "setGyroSF"); // divider=4, 2000dps
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]),
        "ACCEL_ONLY_GAIN");
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]),
        "ACCEL_ALPHA_VAR");
  const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  check(icm_.writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]), "ACCEL_A_VAR");

  // Use Quat9 (9-axis orientation vector, includes magnetometer). Yaw drift is
  // acceptable for a self-balancing robot, and Quat9 is not rate-limited by
  // the magnetometer ODR (68.75Hz) that caps Quat9 at ~56Hz.
  check(icm_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION), "enableDMPSensor");
  check(icm_.setDMPODRrate(DMP_ODR_Reg_Quat9, 0), "setDMPODRrate");

  check(icm_.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE), "enableDMPSensor");
  check(icm_.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0), "setDMPODRrate");

  check(icm_.enableFIFO(), "enableFIFO");
  check(icm_.enableDMP(), "enableDMP");
  check(icm_.resetDMP(), "resetDMP");
  check(icm_.resetFIFO(), "resetFIFO");

  return success;
}

void transformToReferenceFrame(IMU::Data &data) {
  // Normalize axes: hardware (+X back, +Y left) → sim (+X forward, +Y right)
  data.gyr[0] = -data.gyr[0];
  data.gyr[1] = -data.gyr[1];

  data.orientation[0] = -data.orientation[0];
  data.orientation[1] = -data.orientation[1];
}

bool IMU::try_read(IMU::Data &out_data) {
  // gpm4 and dps2000 are hardcoded in initializeDMP
  constexpr float quat9_scale = 1.0f / 1073741824.0f; // 1 / 2^30
  constexpr float accel_scale = 9.80665f / 8192.0f;   // gpm4: m/s² per LSB
  constexpr float gyro_scale =
      (M_PI / 180.0f) / 2048.0f; // dps2000: rad/s per LSB

  constexpr uint8_t kMaxDrain = 30; // Avoid draining the FIFO too much in one
                                    // go, to give the DMP a chance to refill it
  uint8_t drain_count = 0;

  bool got_data = false;
  icm_.readDMPdataFromFIFO(&data_dmp_);

  while (drain_count++ < kMaxDrain &&
         ((icm_.status == ICM_20948_Stat_Ok) ||
          (icm_.status == ICM_20948_Stat_FIFOMoreDataAvail))) {
    if (data_dmp_.header & DMP_header_bitmap_Quat9) {
      float q1 = data_dmp_.Quat9.Data.Q1 * quat9_scale;
      float q2 = data_dmp_.Quat9.Data.Q2 * quat9_scale;
      float q3 = data_dmp_.Quat9.Data.Q3 * quat9_scale;
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

  if (drain_count >= kMaxDrain) {
    DPRINTLN("Warning: IMU FIFO overflow, resetting FIFO");
    icm_.resetFIFO();
    got_data = false; // discard data if we had to reset the FIFO
  }

  if (got_data) {
    transformToReferenceFrame(out_data);
  }
  return got_data;
}

void IMU::set_dmp_biases(const IMU::Calibration &cal) {
  icm_.setBiasAccelX(cal.accel[0]);
  icm_.setBiasAccelY(cal.accel[1]);
  icm_.setBiasAccelZ(cal.accel[2]);

  icm_.setBiasGyroX(cal.gyro[0]);
  icm_.setBiasGyroY(cal.gyro[1]);
  icm_.setBiasGyroZ(cal.gyro[2]);

  icm_.setBiasCPassX(cal.mag[0]);
  icm_.setBiasCPassY(cal.mag[1]);
  icm_.setBiasCPassZ(cal.mag[2]);
}

IMU::Calibration IMU::get_dmp_biases() {
  IMU::Calibration cal;
  icm_.getBiasAccelX(&cal.accel[0]);
  icm_.getBiasAccelY(&cal.accel[1]);
  icm_.getBiasAccelZ(&cal.accel[2]);

  icm_.getBiasGyroX(&cal.gyro[0]);
  icm_.getBiasGyroY(&cal.gyro[1]);
  icm_.getBiasGyroZ(&cal.gyro[2]);

  icm_.getBiasCPassX(&cal.mag[0]);
  icm_.getBiasCPassY(&cal.mag[1]);
  icm_.getBiasCPassZ(&cal.mag[2]);
  return cal;
}

void IMU::print_biases() {
  IMU::Calibration cal = get_dmp_biases();
  DPRINTLN("Current biases:");
  DPRINTF("  Accel: [%d, %d, %d]\n", cal.accel[0], cal.accel[1], cal.accel[2]);
  DPRINTF("  Gyro:  [%d, %d, %d]\n", cal.gyro[0], cal.gyro[1], cal.gyro[2]);
  DPRINTF("  Mag:   [%d, %d, %d]\n", cal.mag[0], cal.mag[1], cal.mag[2]);
}

IMU::Calibration IMU::save_biases() {
  IMU::Calibration cal = get_dmp_biases();

  Preferences prefs;
  prefs.begin(kNvsNamespace, false);
  prefs.putBytes(kNvsKeyGyro, cal.gyro, sizeof(cal.gyro));
  prefs.putBytes(kNvsKeyAccel, cal.accel, sizeof(cal.accel));
  prefs.putBytes(kNvsKeyMag, cal.mag, sizeof(cal.mag));
  prefs.end();

  return cal;
}

IMU::Calibration IMU::load_biases() {
  IMU::Calibration cal = {
      .gyro = {kDefaultBiasGyro[0], kDefaultBiasGyro[1], kDefaultBiasGyro[2]},
      .accel = {kDefaultBiasAccel[0], kDefaultBiasAccel[1],
                kDefaultBiasAccel[2]},
      .mag = {kDefaultBiasMag[0], kDefaultBiasMag[1], kDefaultBiasMag[2]}};

  Preferences prefs;
  if (false && prefs.begin(kNvsNamespace, true)) { // read-only
    prefs.getBytes(kNvsKeyGyro, cal.gyro, sizeof(cal.gyro));
    prefs.getBytes(kNvsKeyAccel, cal.accel, sizeof(cal.accel));
    prefs.getBytes(kNvsKeyMag, cal.mag, sizeof(cal.mag));
    prefs.end();
    DPRINTLN("IMU biases loaded from NVS");
  } else {
    DPRINTLN("IMU biases: NVS empty, using factory defaults");
  }

  set_dmp_biases(cal);
  return get_dmp_biases();
}