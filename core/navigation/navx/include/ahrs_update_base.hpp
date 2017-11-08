#ifndef  AHRS_UPDATE_BASE_HPP
#define AHRS_UPDATE_BASE_HPP

#include <stdint.h>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class AhrsUpdateBase
            {
            public:
                float yaw;
                float pitch;
                float roll;
                float compass_heading;
                float altitude;
                float fused_heading;
                float linear_accel_x;
                float linear_accel_y;
                float linear_accel_z;
                float quat_w;
                float quat_x;
                float quat_y;
                float quat_z;
                float baro_pressure;
                float baro_temp;
                uint8_t op_status;
                uint8_t sensor_status;
                uint8_t cal_status;
                uint8_t selftest_status;
            };
        } // navx
    } // navigation
} // rip

#endif // AHRS_UPDATE_BASE_HPP
