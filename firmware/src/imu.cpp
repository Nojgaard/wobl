#include <imu.hpp>

IMU::IMU() {}

bool IMU::initialize()
{
    if (icm_.begin(Wire, true) != ICM_20948_Stat_Ok)
        return false;
    return true;
    bool success = true;

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (icm_.initializeDMP() != ICM_20948_Stat_Ok);

    success &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) != ICM_20948_Stat_Ok);
    success &= (icm_.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) != ICM_20948_Stat_Ok);

    success &= (icm_.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (icm_.enableDMP() == ICM_20948_Stat_Ok);
    success &= (icm_.resetDMP() == ICM_20948_Stat_Ok);
    success &= (icm_.resetFIFO() == ICM_20948_Stat_Ok);

    return success;
}

bool IMU::try_read()
{
    if (icm_.status != ICM_20948_Stat_Ok)
        return false;

    icm_.readDMPdataFromFIFO(&data_dmp_);

    ICM_20948_Status_e result = icm_.readDMPdataFromFIFO(&data_dmp_);
    if (result != ICM_20948_Stat_Ok && result != ICM_20948_Stat_FIFOMoreDataAvail)
        return false;

    while ((icm_.status == ICM_20948_Stat_Ok) || (icm_.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        if ((data_dmp_.header & DMP_header_bitmap_Quat9) > 0)
        {
            // Scale to +/- 1
            double q1 = ((double)data_dmp_.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data_dmp_.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data_dmp_.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = std::sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        }
        if ((data_dmp_.header & DMP_header_bitmap_Accel) > 0)
        {
            float lsb2g = 1 / 8192.0; // gpm4 is hardcoded in the DMP init function
            float gravity = 9.80665;
            const auto &data = data_dmp_.Raw_Accel.Data;
        }
        if ((data_dmp_.header & DMP_header_bitmap_Gyro_Calibr) > 0)
        {
            float lsb2dps = 1 / 2048.0; // dps2000 is hardcoded in the DMP init function
            float dps2rps = 0.0174532925;
            const auto &data = data_dmp_.Raw_Gyro.Data;
        }
        icm_.readDMPdataFromFIFO(&data_dmp_);
    }

    if (icm_.status == ICM_20948_Stat_FIFONoDataAvail)
        icm_.status = ICM_20948_Stat_Ok; // Clear this to avoid returning false next time

    return true;
}