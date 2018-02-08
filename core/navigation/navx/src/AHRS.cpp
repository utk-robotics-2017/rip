
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <AHRS.h>
#include <AHRSProtocol.h>
#include <ctime>
#include <pthread.h>
#include "IIOProvider.h"
#include "IIOCompleteNotification.h"
#include "IBoardCapabilities.h"
#include "InertialDataIntegrator.h"
#include "OffsetTracker.h"
#include "ContinuousAngleTracker.h"
#include "SerialIO.h"
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            static const uint8_t    NAVX_DEFAULT_UPDATE_RATE_HZ         = 60;
            static const int        YAW_HISTORY_LENGTH                  = 10;
            static const int16_t    DEFAULT_ACCEL_FSR_G                 = 2;
            static const int16_t    DEFAULT_GYRO_FSR_DPS                = 2000;
            static const uint32_t   MAX_SPI_BITRATE                     = 2000000;
            static const uint32_t   MIN_SPI_BITRATE                     = 100000;
            static const uint32_t   DEFAULT_SPI_BITRATE                 = 500000;
            static const uint8_t    NAVX_MXP_I2C_ADDRESS                = 0x32;
            static const float      QUATERNION_HISTORY_SECONDS          = 5.0f;

            class AHRSInternal : public IIOCompleteNotification, public IBoardCapabilities
            {
                AHRS *ahrs;
                friend class AHRS;
                AHRSInternal(AHRS* ahrs)
                {
                    this->ahrs = ahrs;
                }

                /***********************************************************/
                /* IIOCompleteNotification Interface Implementation        */
                /***********************************************************/

                void setYawPitchRoll(IMUProtocol::YPRUpdate& ypr_update, long sensor_timestamp)
                {
                    //printf("Setting pitch value to %f", ypr_update.pitch);
                    ahrs->yaw                   = ypr_update.yaw;
                    ahrs->pitch                 = ypr_update.pitch;
                    ahrs->roll                  = ypr_update.roll;
                    ahrs->compass_heading       = ypr_update.compass_heading;
                    ahrs->last_sensor_timestamp = sensor_timestamp;
                }

                void setAHRSPosData(AHRSProtocol::AHRSPosUpdate& ahrs_update, long sensor_timestamp)
                {
                    /* Update base IMU class variables */
                    //TODO: add spdlogging here
                    //printf("Setting pitch to: %f\n", ahrs_update.pitch);
                    ahrs->yaw                    = ahrs_update.yaw;
                    ahrs->pitch                  = ahrs_update.pitch;
                    ahrs->roll                   = ahrs_update.roll;
                    ahrs->compass_heading        = ahrs_update.compass_heading;
                    ahrs->yaw_offset_tracker->updateHistory(ahrs_update.yaw);

                    /* Update AHRS class variables */

                    // 9-axis data
                    ahrs->fused_heading          = ahrs_update.fused_heading;

                    // Gravity-corrected linear acceleration(world-frame)
                    ahrs->world_linear_accel_x   = ahrs_update.linear_accel_x;
                    ahrs->world_linear_accel_y   = ahrs_update.linear_accel_y;
                    ahrs->world_linear_accel_z   = ahrs_update.linear_accel_z;

                    // Gyro/Accelerometer Die Temperature
                    ahrs->mpu_temp_c             = ahrs_update.mpu_temp;

                    // Barometric Pressure/Altitude
                    ahrs->altitude               = ahrs_update.altitude;
                    ahrs->baro_pressure          = ahrs_update.barometric_pressure;

                    // Status/Motion Detection
                    ahrs->is_moving              =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_MOVING) != 0)
                                    ? true : false);
                    ahrs->is_rotating                =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
                                    ? false : true);
                    ahrs->altitude_valid             =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
                                    ? true : false);
                    ahrs->is_magnetometer_calibrated =
                           (((ahrs_update.cal_status &
                                    NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
                                    ? true : false);
                    ahrs->magnetic_disturbance       =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
                                    ? true : false);

                    ahrs->quaternionW                = ahrs_update.quat_w;
                    ahrs->quaternionX                = ahrs_update.quat_x;
                    ahrs->quaternionY                = ahrs_update.quat_y;
                    ahrs->quaternionZ                = ahrs_update.quat_z;

                    ahrs->last_sensor_timestamp = sensor_timestamp;

                    /* Notify external data arrival subscribers, if any. */
                    for(int i = 0; i < MAX_NUM_CALLBACKS; i++)
                    {
                        ITimestampedDataSubscriber *callback = ahrs->callbacks[i];
                        if(callback != NULL)
                        {
                            long system_timestamp =(long)(std::time(nullptr) * 1000);
                            callback->timestampedDataReceived(system_timestamp,
                                    sensor_timestamp,
                                    ahrs_update,
                                    ahrs->callback_contexts[i]);
                        }
                    }

                    ahrs->velocity[0]     = ahrs_update.vel_x;
                    ahrs->velocity[1]     = ahrs_update.vel_y;
                    ahrs->velocity[2]     = ahrs_update.vel_z;
                    ahrs->displacement[0] = ahrs_update.disp_x;
                    ahrs->displacement[1] = ahrs_update.disp_y;
                    ahrs->displacement[2] = ahrs_update.disp_z;

                    ahrs->yaw_angle_tracker->nextAngle((ahrs->getYaw())());
                    ahrs->last_sensor_timestamp = sensor_timestamp;
                }

                void setRawData(AHRSProtocol::GyroUpdate& raw_data_update, long sensor_timestamp)
                {
                    ahrs->raw_gyro_x     = raw_data_update.gyro_x;
                    ahrs->raw_gyro_y     = raw_data_update.gyro_y;
                    ahrs->raw_gyro_z     = raw_data_update.gyro_z;
                    ahrs->raw_accel_x    = raw_data_update.accel_x;
                    ahrs->raw_accel_y    = raw_data_update.accel_y;
                    ahrs->raw_accel_z    = raw_data_update.accel_z;
                    ahrs->cal_mag_x      = raw_data_update.mag_x;
                    ahrs->cal_mag_y      = raw_data_update.mag_y;
                    ahrs->cal_mag_z      = raw_data_update.mag_z;
                    ahrs->mpu_temp_c     = raw_data_update.temp_c;
                    ahrs->last_sensor_timestamp = sensor_timestamp;
                }

                void setAHRSData(AHRSProtocol::AHRSUpdate& ahrs_update, long sensor_timestamp)
                {
                    /* Update base IMU class variables */

                    ahrs->yaw                    = ahrs_update.yaw;
                    ahrs->pitch                  = ahrs_update.pitch;
                    ahrs->roll                   = ahrs_update.roll;
                    ahrs->compass_heading        = ahrs_update.compass_heading;
                    ahrs->yaw_offset_tracker->updateHistory(ahrs_update.yaw);

                    /* Update AHRS class variables */

                    // 9-axis data
                    ahrs->fused_heading          = ahrs_update.fused_heading;

                    // Gravity-corrected linear acceleration(world-frame)
                    ahrs->world_linear_accel_x   = ahrs_update.linear_accel_x;
                    ahrs->world_linear_accel_y   = ahrs_update.linear_accel_y;
                    ahrs->world_linear_accel_z   = ahrs_update.linear_accel_z;

                    // Gyro/Accelerometer Die Temperature
                    ahrs->mpu_temp_c             = ahrs_update.mpu_temp;

                    // Barometric Pressure/Altitude
                    ahrs->altitude               = ahrs_update.altitude;
                    ahrs->baro_pressure          = ahrs_update.barometric_pressure;

                    // Magnetometer Data
                    ahrs->cal_mag_x              = ahrs_update.cal_mag_x;
                    ahrs->cal_mag_y              = ahrs_update.cal_mag_y;
                    ahrs->cal_mag_z              = ahrs_update.cal_mag_z;

                    // Status/Motion Detection
                    ahrs->is_moving              =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_MOVING) != 0)
                                    ? true : false);
                    ahrs->is_rotating                =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
                                    ? false : true);
                    ahrs->altitude_valid             =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
                                    ? true : false);
                    ahrs->is_magnetometer_calibrated =
                           (((ahrs_update.cal_status &
                                    NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
                                    ? true : false);
                    ahrs->magnetic_disturbance       =
                           (((ahrs_update.sensor_status &
                                    NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
                                    ? true : false);

                    ahrs->quaternionW                = ahrs_update.quat_w;
                    ahrs->quaternionX                = ahrs_update.quat_x;
                    ahrs->quaternionY                = ahrs_update.quat_y;
                    ahrs->quaternionZ                = ahrs_update.quat_z;

                    ahrs->last_sensor_timestamp = sensor_timestamp;

                    /* Notify external data arrival subscribers, if any. */
                    for(int i = 0; i < MAX_NUM_CALLBACKS; i++)
                    {
                        ITimestampedDataSubscriber *callback = ahrs->callbacks[i];
                        if(callback != NULL)
                        {
                            long system_timestamp =(long)(std::time(nullptr) * 1000);
                            callback->timestampedDataReceived(system_timestamp,
                                    sensor_timestamp,
                                    ahrs_update,
                                    ahrs->callback_contexts[i]);
                        }
                    }

                    ahrs->updateDisplacement(ahrs->world_linear_accel_x,
                            ahrs->world_linear_accel_y,
                            ahrs->update_rate_hz,
                            ahrs->is_moving);

                    ahrs->yaw_angle_tracker->nextAngle((ahrs->getYaw())());
                }

                void setBoardID(AHRSProtocol::BoardID& board_id)
                {
                    ahrs->board_type = board_id.type;
                    ahrs->hw_rev = board_id.hw_rev;
                    ahrs->fw_ver_major = board_id.fw_ver_major;
                    ahrs->fw_ver_minor = board_id.fw_ver_minor;
                }

                void setBoardState(IIOCompleteNotification::BoardState& board_state)
                {
                    ahrs->update_rate_hz = board_state.update_rate_hz;
                    ahrs->accel_fsr_g = board_state.accel_fsr_g;
                    ahrs->gyro_fsr_dps = board_state.gyro_fsr_dps;
                    ahrs->capability_flags = board_state.capability_flags;
                    ahrs->op_status = board_state.op_status;
                    ahrs->sensor_status = board_state.sensor_status;
                    ahrs->cal_status = board_state.cal_status;
                    ahrs->selftest_status = board_state.selftest_status;
                 }

                /***********************************************************/
                /* IBoardCapabilities Interface Implementation        */
                /***********************************************************/
                bool isOmniMountSupported()
                {
                   return(((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_OMNIMOUNT) !=0) ? true : false);
                }

                bool isBoardYawResetSupported()
                {
                    return(((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_YAW_RESET) != 0) ? true : false);
                }

                bool isDisplacementSupported()
                {
                    return(((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0) ? true : false);
                }

                bool isAHRSPosTimestampSupported()
                {
                    return(((ahrs->capability_flags & NAVX_CAPABILITY_FLAG_AHRSPOS_TS) != 0) ? true : false);
                }
            };

            AHRS::AHRS(std::string serial_port_id, AHRS::serialDataType data_type, uint8_t update_rate_hz)
            {
                serialInit(serial_port_id, data_type, update_rate_hz);
            }

            AHRS::AHRS(std::string serial_port_id)
            {
                serialInit(serial_port_id, serialDataType::kProcessedData, NAVX_DEFAULT_UPDATE_RATE_HZ);
            }

            units::Angle AHRS::getPitch()
            {
                return static_cast<float>(pitch)*units::deg;
            }

            units::Angle AHRS::getRoll()
            {
                return static_cast<float>(roll)*units::degrees;
            }

            units::Angle AHRS::getYaw()
            {
                if(ahrs_internal->isBoardYawResetSupported())
                {
                    return static_cast<float>((this->yaw))*units::degrees;
                }
                else
                {
                    return((float) yaw_offset_tracker->applyOffset(this->yaw))*units::degrees;
                }
            }

            units::Angle AHRS::getCompassHeading()
            {
                return static_cast<float>(compass_heading)*units::degrees;
            }

            void AHRS::zeroYaw()
            {
                if(ahrs_internal->isBoardYawResetSupported())
                {
                    io->zeroYaw();
                }
                else
                {
                    yaw_offset_tracker->setOffset();
                }
            }

            bool AHRS::isCalibrating()
            {
                return !((cal_status &
                            NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) ==
                                NAVX_CAL_STATUS_IMU_CAL_COMPLETE);
            }

            bool AHRS::isConnected()
            {
                return io->isConnected();
            }

            double AHRS::getByteCount()
            {
                return io->getByteCount();
            }

            double AHRS::getUpdateCount()
            {
                return io->getUpdateCount();
            }

            long AHRS::getLastSensorTimestamp()
            {
                return this->last_sensor_timestamp;
            }

            units::Acceleration AHRS::getWorldLinearAccelX()
            {
                return static_cast<double>(this->world_linear_accel_x) * units::AccelerationOfGravity;
            }

            units::Acceleration AHRS::getWorldLinearAccelY()
            {
                return static_cast<double>(this->world_linear_accel_y) * units::AccelerationOfGravity;
            }

            units::Acceleration AHRS::getWorldLinearAccelZ()
            {
                return static_cast<double>(this->world_linear_accel_z) * units::AccelerationOfGravity;
            }

            bool AHRS::isMoving()
            {
                return is_moving;
            }

            bool AHRS::isRotating()
            {
                return is_rotating;
            }

            units::Pressure AHRS::getBarometricPressure()
            {
                return static_cast<float>(baro_pressure) * units::millibar;
            }

            units::Distance AHRS::getAltitude()
            {
                return static_cast<float>(altitude) * units::m;
            }

            bool AHRS::isAltitudeValid()
            {
                return this->altitude_valid;
            }

            units::Angle AHRS::getFusedHeading()
            {
                return static_cast<float>(fused_heading) * units::degrees;
            }

            bool AHRS::isMagneticDisturbance()
            {
                return magnetic_disturbance;
            }

            bool AHRS::isMagnetometerCalibrated()
            {
                return is_magnetometer_calibrated;
            }

            float AHRS::getQuaternionW()
            {
                return quaternionW;
            }

            float AHRS::getQuaternionX()
            {
                return quaternionX;
            }

            float AHRS::getQuaternionY()
            {
                return quaternionY;
            }

            float AHRS::getQuaternionZ()
            {
                return quaternionZ;
            }

            void AHRS::resetDisplacement()
            {
                if(ahrs_internal->isDisplacementSupported())
                {
                    io->zeroDisplacement();
                }
                else
                {
                    integrator->resetDisplacement();
                }
            }

            void AHRS::updateDisplacement(float accel_x_g, float accel_y_g,
                                                int update_rate_hz, bool is_moving)
            {
                integrator->updateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
            }


            units::Velocity AHRS::getVelocityX()
            {
                return units::m / units::s *(ahrs_internal->isDisplacementSupported() ? velocity[0] : integrator->getVelocityX());
            }


            units::Velocity AHRS::getVelocityY()
            {
                return units::m / units::s *(ahrs_internal->isDisplacementSupported() ? velocity[1] : integrator->getVelocityY());
            }


            units::Velocity AHRS::getVelocityZ()
            {
                return units::m / units::s *(ahrs_internal->isDisplacementSupported() ? velocity[2] : 0.f);
            }


            units::Distance AHRS::getDisplacementX()
            {
                return units::m *(ahrs_internal->isDisplacementSupported() ? displacement[0] : integrator->getVelocityX());
            }

            units::Distance AHRS::getDisplacementY()
            {
                return units::m *(ahrs_internal->isDisplacementSupported() ? displacement[1] : integrator->getVelocityY());
            }

            units::Distance AHRS::getDisplacementZ()
            {
                return units::m *(ahrs_internal->isDisplacementSupported() ? displacement[2] : 0.f);
            }

            #define NAVX_IO_THREAD_NAME "navXIOThread"


            void AHRS::serialInit(std::string serial_port_id, AHRS::serialDataType data_type, uint8_t update_rate_hz)
            {
                commonInit(update_rate_hz);
                bool processed_data =(data_type == serialDataType::kProcessedData);
                io = new SerialIO(serial_port_id, update_rate_hz, processed_data, ahrs_internal, ahrs_internal);
                ::pthread_t trd;
                ::pthread_create(&trd, NULL, AHRS::threadFunc, io);
            }

            void AHRS::commonInit(uint8_t update_rate_hz)
            {

                ahrs_internal = new AHRSInternal(this);
                this->update_rate_hz = update_rate_hz;

                /* Processed Data */

                yaw_offset_tracker = new OffsetTracker(YAW_HISTORY_LENGTH);
                integrator = new InertialDataIntegrator();
                yaw_angle_tracker = new ContinuousAngleTracker();

                yaw =
                        pitch =
                                roll =
                                        compass_heading = 0.0f;
                world_linear_accel_x =
                        world_linear_accel_y =
                                world_linear_accel_z = 0.0f;
                mpu_temp_c = 0.0f;
                fused_heading = 0.0f;
                altitude = 0.0f;
                baro_pressure = 0.0f;
                is_moving = false;
                is_rotating = false;
                baro_sensor_temp_c = 0.0f;
                altitude_valid = false;
                is_magnetometer_calibrated = false;
                magnetic_disturbance = false;
                quaternionW =
                        quaternionX =
                                quaternionY =
                                        quaternionZ = 0.0f;

                /* Integrated Data */

                for(int i = 0; i < 3; i++)
                {
                    velocity[i] = 0.0f;
                    displacement[i] = 0.0f;
                }


                /* Raw Data */
                raw_gyro_x =
                        raw_gyro_y =
                                raw_gyro_z = 0.0f;
                raw_accel_x =
                        raw_accel_y =
                                raw_accel_z = 0.0f;
                cal_mag_x =
                        cal_mag_y =
                                cal_mag_z = 0.0f;

                /* Configuration/Status */
                update_rate_hz = 0;
                accel_fsr_g = DEFAULT_ACCEL_FSR_G;
                gyro_fsr_dps = DEFAULT_GYRO_FSR_DPS;
                capability_flags = 0;
                op_status =
                        sensor_status =
                                cal_status =
                                        selftest_status = 0;
                /* Board ID */
                board_type =
                        hw_rev =
                                fw_ver_major =
                                        fw_ver_minor = 0;
                last_sensor_timestamp = 0;
                last_update_time = 0;

                io = 0;

                for(int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    callbacks[i] = NULL;
                    callback_contexts[i] = NULL;
                }
            }

            units::Angle AHRS::getAngle()
            {
                return yaw_angle_tracker->getAngle() * units::deg;
            }

            units::AngularVelocity AHRS::getRate()
            {
                return yaw_angle_tracker->getRate() * units::degrees / units::s;
            }

            void AHRS::reset()
            {
                zeroYaw();
            }

            static const float DEV_UNITS_MAX = 32768.0f;

            float AHRS::getRawGyroX()
            {
                return this->raw_gyro_x /(DEV_UNITS_MAX /(float)gyro_fsr_dps);
            }

            float AHRS::getRawGyroY()
            {
                return this->raw_gyro_y /(DEV_UNITS_MAX /(float)gyro_fsr_dps);
            }

            float AHRS::getRawGyroZ()
            {
                return this->raw_gyro_z /(DEV_UNITS_MAX /(float)gyro_fsr_dps);
            }

            float AHRS::getRawAccelX()
            {
                return this->raw_accel_x /(DEV_UNITS_MAX /(float)accel_fsr_g);
            }

            float AHRS::getRawAccelY()
            {
                return this->raw_accel_y /(DEV_UNITS_MAX /(float)accel_fsr_g);
            }


            float AHRS::getRawAccelZ()
            {
                return this->raw_accel_z /(DEV_UNITS_MAX /(float)accel_fsr_g);
            }

            static const float UTESLA_PER_DEV_UNIT = 0.15f;

            float AHRS::getRawMagX()
            {
                return this->cal_mag_x / UTESLA_PER_DEV_UNIT;
            }

            float AHRS::getRawMagY()
            {
                return this->cal_mag_y / UTESLA_PER_DEV_UNIT;
            }

            float AHRS::getRawMagZ()
            {
                return this->cal_mag_z / UTESLA_PER_DEV_UNIT;
            }

            units::Temperature AHRS::getTempC()
            {
                return static_cast<float>(this->mpu_temp_c) * units::degC;
            }

            AHRS::BoardYawAxis AHRS::getBoardYawAxis()
            {
                BoardYawAxis yaw_axis;
                short yaw_axis_info =(short)(capability_flags >> 3);
                yaw_axis_info &= 7;
                if(yaw_axis_info == OMNIMOUNT_DEFAULT)
                {
                    yaw_axis.up = true;
                    yaw_axis.board_axis = boardAxis::kBoardAxisZ;
                }
                else
                {
                    yaw_axis.up =(((yaw_axis_info & 0x01) != 0) ? true : false);
                    yaw_axis_info >>= 1;
                    switch(yaw_axis_info)
                    {
                    case 0:
                        yaw_axis.board_axis = boardAxis::kBoardAxisX;
                        break;
                    case 1:
                        yaw_axis.board_axis = boardAxis::kBoardAxisY;
                        break;
                    case 2:
                    default:
                        yaw_axis.board_axis = boardAxis::kBoardAxisZ;
                        break;
                    }
                }
                return yaw_axis;
            }

            std::string AHRS::getFirmwareVersion()
            {
                std::ostringstream os;
                os <<(int)fw_ver_major << "." <<(int)fw_ver_minor;
                std::string fw_version = os.str();
                return fw_version;
            }

            void *AHRS::threadFunc(void *threadarg)
            {
                IIOProvider *io_provider =(IIOProvider*)threadarg;
                io_provider->run();
                return NULL;
            }

            bool AHRS::registerCallback(ITimestampedDataSubscriber *callback, void *callback_context)
            {
                bool registered = false;
                for(int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    if(callbacks[i] == NULL)
                    {
                        callbacks[i] = callback;
                        callback_contexts[i] = callback_context;
                        registered = true;
                        break;
                    }
                }
                return registered;
            }

            bool AHRS::deregisterCallback(ITimestampedDataSubscriber *callback)
            {
                bool deregistered = false;
                for(int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    if(callbacks[i] == callback)
                    {
                        callbacks[i] = NULL;
                        deregistered = true;
                        break;
                    }
                }
                return deregistered;
            }

            int AHRS::getActualUpdateRate()
            {
                uint8_t actual_update_rate = getActualUpdateRateInternal(getRequestedUpdateRate());
                return(int)actual_update_rate;
            }

            uint8_t AHRS::getActualUpdateRateInternal(uint8_t update_rate)
            {
            #define NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ 200
                int integer_update_rate =(int)update_rate;
                int realized_update_rate = NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ /
                       (NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ / integer_update_rate);
                return(uint8_t)realized_update_rate;
            }

            int AHRS::getRequestedUpdateRate()
            {
                return(int)update_rate_hz;
            }

            void AHRS::close()
            {
                io->stop();
            }
        }
    }
}
