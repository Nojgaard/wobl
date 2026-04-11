#include <imu.hpp>
#include "debug.hpp"

// Override of the weak ICM_20948::initializeDMP from the SparkFun library.
//
// WHAT CHANGED vs the library default (divider=19, ~56Hz):
//   The library only has three sets of self-consistent DMP constants,
//   corresponding to the three rates documented in InvenSense's confidential
//   Application Note:
//     divider=19 -> ~56Hz  (library default)
//     divider=8  -> ~112Hz (this override)
//     divider=4  -> ~225Hz
//   Any other divider (e.g. 10 for ~100Hz) has no published constants and is
//   unsafe.
//
//   The four places that must change together when changing rate:
//     1. mySmplrt.g / mySmplrt.a  — hardware sample rate divider register
//     2. setGyroSF(div, level)     — DMP gyro scale factor; converts raw LSBs
//     to
//                                    angle-per-step. Wrong value -> quaternion
//                                    rotates at the wrong speed (proportional
//                                    to rate error).
//     3. ACCEL_ONLY_GAIN           — accel correction gain in the complementary
//     filter.
//                                    Wrong value -> poor levelling when
//                                    stationary.
//     4. ACCEL_ALPHA_VAR /         — exponential smoothing weights for the
//     complementary
//        ACCEL_A_VAR                 filter. Wrong values ->
//        over/under-trusting accel,
//                                    causing jitter or drift.
//
//   The magic byte values for (3) and (4) originate from InvenSense's internal
//   documentation and are available verbatim as commented-out alternatives in
//   the SparkFun library's ICM_20948.cpp (search for "112Hz").
//
// UNCHANGED vs the library: all register configuration, FSR settings, mount
// matrices, firmware loading, and compass setup are identical.
ICM_20948_Status_e ICM_20948::initializeDMP(void) {
  if (_device._dmp_firmware_available != true)
    return ICM_20948_Stat_DMPNotSupported;

#if defined(ICM_20948_USE_DMP)
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

  // Configure magnetometer for DMP (single-measurement mode via I2C master)
  // Unchanged from library default.
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR,
                                            AK09916_REG_RSV2, 10, true, true,
                                            false, true, true);
  if (result > worstResult)
    worstResult = result;
  result = i2cControllerConfigurePeripheral(
      1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false,
      false, AK09916_mode_single);
  if (result > worstResult)
    worstResult = result;

  // I2C master ODR: 1100 / 2^4 = 68.75Hz. Unchanged from library default.
  result = setBank(3);
  if (result > worstResult)
    worstResult = result;
  uint8_t mstODRconfig = 0x04;
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
  if (result > worstResult)
    worstResult = result;

  result = setClockSource(ICM_20948_Clock_Auto);
  if (result > worstResult)
    worstResult = result;

  result = setBank(0);
  if (result > worstResult)
    worstResult = result;
  uint8_t pwrMgmt2 = 0x40;
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);
  if (result > worstResult)
    worstResult = result;

  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
  if (result > worstResult)
    worstResult = result;
  result = enableFIFO(false);
  if (result > worstResult)
    worstResult = result;
  result = enableDMP(false);
  if (result > worstResult)
    worstResult = result;

  // FSR: accel=4g, gyro=2000dps. Unchanged from library default.
  // These must match the DMP scale constants (ACC_SCALE, GYRO_FULLSCALE)
  // written below.
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;
  myFSS.g = dps2000;
  result =
      setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (result > worstResult)
    worstResult = result;
  result = enableDLPF(ICM_20948_Internal_Gyr, true);
  if (result > worstResult)
    worstResult = result;

  // Clear FIFO inputs. Unchanged from library default.
  result = setBank(0);
  if (result > worstResult)
    worstResult = result;
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1);
  if (result > worstResult)
    worstResult = result;
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1);
  if (result > worstResult)
    worstResult = result;
  result = intEnableRawDataReady(false);
  if (result > worstResult)
    worstResult = result;
  result = resetFIFO();
  if (result > worstResult)
    worstResult = result;

  // CHANGE 1/4: Sample rate divider.
  // Library default: mySmplrt.g/a = 19  -> 1100/(1+19) = 55Hz,  1125/(1+19) =
  // 56Hz This override:   mySmplrt.g/a = 8   -> 1100/(1+8)  = 122Hz, 1125/(1+8)
  // = 125Hz
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 8;
  mySmplrt.a = 8;
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                         mySmplrt);
  if (result > worstResult)
    worstResult = result;

  result = setDMPstartAddress();
  if (result > worstResult)
    worstResult = result;
  result = loadDMPFirmware();
  if (result > worstResult)
    worstResult = result;
  result = setDMPstartAddress();
  if (result > worstResult)
    worstResult = result;

  result = setBank(0);
  if (result > worstResult)
    worstResult = result;
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1);
  if (result > worstResult)
    worstResult = result;

  result = setBank(0);
  if (result > worstResult)
    worstResult = result;
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
  if (result > worstResult)
    worstResult = result;

  // Accel DMP scaling for gpm4. Unchanged from library default.
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]);
  if (result > worstResult)
    worstResult = result;

  // Compass and B2S mount matrices. Unchanged from library default.
  const unsigned char zero4[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char plus[4] = {0x09, 0x99, 0x99, 0x99};
  const unsigned char minus[4] = {0xF6, 0x66, 0x66, 0x67};
  result = writeDMPmems(CPASS_MTX_00, 4, &plus[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &minus[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &minus[0]);
  if (result > worstResult)
    worstResult = result;

  const unsigned char b2sPlus[4] = {0x40, 0x00, 0x00, 0x00};
  result = writeDMPmems(B2S_MTX_00, 4, &b2sPlus[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sPlus[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &zero4[0]);
  if (result > worstResult)
    worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sPlus[0]);
  if (result > worstResult)
    worstResult = result;

  // CHANGE 2/4: Gyro scale factor.
  // Converts raw gyro LSBs to the DMP's internal angle-per-step unit.
  // Must match the hardware divider (arg 1) and FSR (arg 2: 3=2000dps).
  // Library default: setGyroSF(19, 3)  -- calibrated for 55Hz
  // This override:   setGyroSF(8, 3)   -- calibrated for 112Hz
  // Source: InvenSense Application Note via SparkFun library setGyroSF
  // implementation.
  result = setGyroSF(8, 3);
  if (result > worstResult)
    worstResult = result;

  // Gyro full-scale register for DMP (2000dps = 2^28). Unchanged from library
  // default.
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00};
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
  if (result > worstResult)
    worstResult = result;

  // CHANGE 3/4: Accel-only correction gain.
  // Controls how aggressively the accel corrects gyro drift when the robot is
  // near-stationary. Values originate from InvenSense's confidential
  // Application Note; the 56Hz/112Hz/225Hz variants are available as
  // commented-out alternatives in the SparkFun library source. Library default:
  // {0x03, 0xA4, 0x92, 0x49}  (56Hz,  = 61117001 decimal) This override: {0x01,
  // 0xD1, 0x74, 0x5D}  (112Hz, = 30504029 decimal)
  const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D};
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
  if (result > worstResult)
    worstResult = result;

  // CHANGE 4/4: Accel complementary filter smoothing weights.
  // These two values set the exponential smoothing time constants for the
  // accel-based tilt correction inside the DMP's fusion algorithm. Matching the
  // rate prevents the filter from being too slow (drifts) or too fast
  // (jittery). Values from InvenSense Application Note via SparkFun library
  // commented-out alternatives. Library default: AlphaVar={0x34,0x92,0x49,0x25}
  // AVar={0x0B,0x6D,0xB6,0xDB}  (56Hz) This override:
  // AlphaVar={0x3A,0x49,0x24,0x92} AVar={0x05,0xB6,0xDB,0x6E}  (112Hz)
  const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92};
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
  if (result > worstResult)
    worstResult = result;

  const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E};
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
  if (result > worstResult)
    worstResult = result;

  // Accel calibration rate and compass time buffer. Unchanged from library
  // default.
  const unsigned char accelCalRate[2] = {0x00, 0x00};
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
  if (result > worstResult)
    worstResult = result;

  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
  if (result > worstResult)
    worstResult = result;

  return worstResult;
#else
  return ICM_20948_Stat_DMPNotSupported;
#endif
}

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

  check(icm_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION),
        "enableDMPSensor");
  check(icm_.setDMPODRrate(DMP_ODR_Reg_Quat9, 0), "setDMPODRrate");
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