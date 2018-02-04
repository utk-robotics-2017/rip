
#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include "ITimestampedDataSubscriber.h"
#include <memory>
#include <string>
#include <units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IIOProvider;
            class ContinuousAngleTracker;
            class InertialDataIntegrator;
            class OffsetTracker;
            class AHRSInternal;

            class AHRS{
            public:

                enum BoardAxis {
                    kBoardAxisX = 0,
                    kBoardAxisY = 1,
                    kBoardAxisZ = 2,
                };

                struct BoardYawAxis
                {
                    /* Identifies one of the board axes */
                    BoardAxis board_axis;
                    /* true if axis is pointing up (with respect to gravity); false if pointing down. */
                    bool up;
                };

                enum SerialDataType {
                /**
                 * (default):  6 and 9-axis processed data
                 */
                kProcessedData = 0,
                /**
                 * unprocessed data from each individual sensor
                 */
                kRawData = 1
                };

            private:
                friend class AHRSInternal;
                AHRSInternal *      ahrs_internal;

                volatile float      yaw;
                volatile float      pitch;
                volatile float      roll;
                volatile float      compass_heading;
                volatile float      world_linear_accel_x;
                volatile float      world_linear_accel_y;
                volatile float      world_linear_accel_z;
                volatile float      mpu_temp_c;
                volatile float      fused_heading;
                volatile float      altitude;
                volatile float      baro_pressure;
                volatile bool       is_moving;
                volatile bool       is_rotating;
                volatile float      baro_sensor_temp_c;
                volatile bool       altitude_valid;
                volatile bool       is_magnetometer_calibrated;
                volatile bool       magnetic_disturbance;
                volatile float    	quaternionW;
                volatile float    	quaternionX;
                volatile float    	quaternionY;
                volatile float    	quaternionZ;

                /* Integrated Data */
                float velocity[3];
                float displacement[3];


                /* Raw Data */
                volatile int16_t    raw_gyro_x;
                volatile int16_t    raw_gyro_y;
                volatile int16_t    raw_gyro_z;
                volatile int16_t    raw_accel_x;
                volatile int16_t    raw_accel_y;
                volatile int16_t    raw_accel_z;
                volatile int16_t    cal_mag_x;
                volatile int16_t    cal_mag_y;
                volatile int16_t    cal_mag_z;

                /* Configuration/Status */
                volatile uint8_t    update_rate_hz;
                volatile int16_t    accel_fsr_g;
                volatile int16_t    gyro_fsr_dps;
                volatile int16_t    capability_flags;
                volatile uint8_t    op_status;
                volatile int16_t    sensor_status;
                volatile uint8_t    cal_status;
                volatile uint8_t    selftest_status;

                /* Board ID */
                volatile uint8_t    board_type;
                volatile uint8_t    hw_rev;
                volatile uint8_t    fw_ver_major;
                volatile uint8_t    fw_ver_minor;

                long                last_sensor_timestamp;
                double              last_update_time;


                InertialDataIntegrator *integrator;
                ContinuousAngleTracker *yaw_angle_tracker;
                OffsetTracker *         yaw_offset_tracker;
                IIOProvider *           io;


            #define MAX_NUM_CALLBACKS 3
                ITimestampedDataSubscriber *callbacks[MAX_NUM_CALLBACKS];
                void *callback_contexts[MAX_NUM_CALLBACKS];

            public:
                AHRS(std::string serial_port_id);

                AHRS(std::string serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);

                units::Angle getPitch();
                units::Angle getRoll();
                units::Angle getYaw();
                units::Angle getCompassHeading();
                void   zeroYaw();
                bool   isCalibrating();
                bool   isConnected();
                double getByteCount();
                double getUpdateCount();
                long   getLastSensorTimestamp();
                units::Acceleration getWorldLinearAccelX();
                units::Acceleration getWorldLinearAccelY();
                units::Acceleration getWorldLinearAccelZ();
                bool   isMoving();
                bool   isRotating();
                units::Pressure getBarometricPressure();
                units::Distance getAltitude();
                bool   isAltitudeValid();
                float  getFusedHeading();
                bool   isMagneticDisturbance();
                bool   isMagnetometerCalibrated();
                float  getQuaternionW();
                float  getQuaternionX();
                float  getQuaternionY();
                float  getQuaternionZ();
                void   resetDisplacement();
                void   updateDisplacement( float accel_x_g, float accel_y_g,
                                           int update_rate_hz, bool is_moving );
                float  getVelocityX();
                float  getVelocityY();
                float  getVelocityZ();
                float  getDisplacementX();
                float  getDisplacementY();
                float  getDisplacementZ();
                double getAngle();
                double getRate();
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
                float  getTempC();
                AHRS::BoardYawAxis getBoardYawAxis();
                std::string getFirmwareVersion();

                bool registerCallback( ITimestampedDataSubscriber *callback, void *callback_context);
                bool deregisterCallback( ITimestampedDataSubscriber *callback );

                int getActualUpdateRate();
                int getRequestedUpdateRate();

                void close();

            private:
                void serialInit(std::string serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
                void commonInit( uint8_t update_rate_hz );
                static void *threadFunc(void *threadarg);

                /* PIDSource implementation */
                double PIDGet();

                uint8_t getActualUpdateRateInternal(uint8_t update_rate);
            };
        } // navx
    } // navigation
} // rip
#endif /* SRC_AHRS_H_ */
