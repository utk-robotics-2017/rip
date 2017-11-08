#ifndef AHRS_HPP
#define AHRS_HPP

#include <units.h>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class Ahrs
            {
            public:
                enum class BoardAxis
                {
                    kX = 0,
                    kY = 1,
                    kZ = 2
                };

                enum class SerialDataType
                {
                    kProcessedData = 0, //!< (default):  6 and 9-axis processed data
                    kRawData = 1 //!< unprocessed data from each individual sensor
                };

                Ahrs();

                /**
                 * Returns the yaw angle
                 * @returns The yaw angle
                 */
                Angle getYaw();

                /**
                 * Zeros the yaw angle
                 */
                void zeroYaw();

                /**
                 * Returns the pitch angle
                 * @returns The pitch angle
                 */
                Angle getPitch();

                /**
                 * Returns the roll angle
                 * @returns The roll angle
                 */
                Angle getRoll();

                /**
                 * [getCompassHeading description]
                 * @return [description]
                 */
                Angle getCompassHeading();

                bool isCalibrating();
                bool isConnected();
                int getByteCount();
                int getUpdateCount();
                Time   getLastSensorTimestamp();
                Acceleration  getWorldLinearAccelX();
                Acceleration  getWorldLinearAccelY();
                Acceleration  getWorldLinearAccelZ();
                bool   isMoving();
                bool   isRotating();
                float  getBarometricPressure();
                float  getAltitude();
                bool   isAltitudeValid();
                Angle  getFusedHeading();
                bool   isMagneticDisturbance();
                bool   isMagnetometerCalibrated();
                float  getQuaternionW();
                float  getQuaternionX();
                float  getQuaternionY();
                float  getQuaternionZ();
                void   resetDisplacement();
                void   updateDisplacement(Acceleration accel_x_g, Acceleration accel_y_g, int update_rate_hz, bool is_moving);
                Velocity  getVelocityX();
                Velocity  getVelocityY();
                Velocity  getVelocityZ();
                Distance  getDisplacementX();
                Distance  getDisplacementY();
                Distance  getDisplacementZ();
                Angle getAngle();
                double getRate();
                void   setAngleAdjustment(Angle angle);
                Angle getAngleAdjustment();
                void   reset();
                float  getRawGyroX();
                float  getRawGyroY();
                float  getRawGyroZ();
                float  getRawAccelX();
                float  getRawAccelY();
                float  getRawAccelZ();
                float  getRawMagX();
                float  getRawMagY();
                float  getRawMagZ();
                float  getPressure();
                Temperature  getTempC();
                AHRS::BoardYawAxis GetBoardYawAxis();
                std::string GetFirmwareVersion();

                int getActualUpdateRate();
                int getRequestedUpdateRate();

            private:
                volatile float m_yaw;
                volatile float m_pitch;
                volatile float m_roll;
                volatile float m_compass_heading;
                volatile float m_world_linear_accel_x;
                volatile float m_world_linear_accel_y;
                volatile float m_world_linear_accel_z;
                volatile float m_mpu_temp_x;
                volatile float m_fused_heading
                volatile float m_altitude;
                volatile float m_baro_pressure;
                volatile float m_is_moving;
                volatile float m_is_rotating;
                volatile float m_baro_sensor_temp_c;
                volatile float m_altitude_valid;
                volatile float m_is_magnetometer_calibrated;
                volatile float m_magnetic_disturbance;
                volatile float m_quaternion_w;
                volatile float m_quaternion_x;
                volatile float m_quaternion_y;
                volatile float m_quaternion_z;

                // Integrated data
                std::array<float, 3> m_velocity;
                std::array<float, 3> m_displacement;

                // Raw data
                volatile int16_t m_raw_gyro_x;
                volatile int16_t m_raw_gyro_y;
                volatile int16_t m_raw_gyro_z;
                volatile int16_t m_raw_accel_x;
                volatile int16_t m_raw_accel_y;
                volatile int16_t m_raw_accel_z;
                volatile int16_t m_cal_mag_x;
                volatile int16_t m_cal_mag_y;
                volatile int16_t m_cal_mag_z;

                // Configuration/Status
                volatile uint8_t m_update_rate_hz;
                volatile int16_t m_accel_fsr_g;
                volatile int16_t m_gyro_fsr_dps;
                volatile int16_t m_capability_flags;
                volatile uint8_t m_op_status;
                volatile int16_t m_sensor_status;
                volatile uint8_t m_cal_status;
                volatile uint8_t m_selftest_status;

                // Board Id
                volatile uint8_t m_board_type;
                volatile uint8_t m_hw_rev;
                volatile uint8_t m_fw_ver_major;
                volatile uint8_t m_fw_ver_minor;

                long m_last_sensor_timestamp;
                double m_last_update_time;

                std::unique_ptr<InertialDataIntegrator> m_integrator;
                std::unique_ptr<ContinuousAngleTracker> m_yaw_angle_tracker;
                std::unique_ptr<OffsetTracker> m_yaw_offset_tracker;
                std::unique_ptr<IIOProvider> m_io;
                std::unique_ptr<std::thread> m_task;
            };
        } // navx
    } // navigation
} // rip


#endif // AHRS_HPP
