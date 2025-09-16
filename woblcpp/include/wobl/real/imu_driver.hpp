#pragma once

#include <ICM_20948.h>
#include <array>
#include <wobl/msg/imu.pb.h>

namespace wobl::real
{
    class ImuDriver
    {
    public:
        ImuDriver();
        bool initialize();
        bool try_read(msg::Imu &imu_msg);
        //msg::Vector3 bias_linear_acceleration();
        //msg::Vector3 bias_angular_velocity();
        //msg::Vector3 bias_compass();

    private:
        void try_load_bias();
        icm_20948_DMP_data_t data_dmp_;
        ICM_20948 icm_;
    };
}