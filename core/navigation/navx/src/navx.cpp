
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <memory>
#include <iomanip>
#include <navx/navx.hpp>
#include <navx/navx_protocol.hpp>
#include <ctime>
#include <pthread.h>
#include "navx/iio_provider.hpp"
#include "navx/iio_complete_notification.hpp"
#include "navx/i_board_capabilities.hpp"
#include "navx/inertial_data_integrator.hpp"
#include "navx/offset_tracker.hpp"
#include "navx/continuous_angle_tracker.hpp"
#include "navx/serial_io.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>

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

            class NavXInternal : public IIOCompleteNotification, public IBoardCapabilities
            {
                NavX* navx;
                friend class NavX;
                NavXInternal(NavX* navx)
                {
                    this->navx = navx;
                }

                /***********************************************************/
                /* IIOCompleteNotification Interface Implementation        */
                /***********************************************************/

                void setYawPitchRoll(IMUProtocol::YPRUpdate& ypr_update, long sensor_timestamp)
                {
                    //printf("Setting pitch value to %f", ypr_update.pitch);
                    navx->yaw                   = ypr_update.yaw;
                    navx->pitch                 = ypr_update.pitch;
                    navx->roll                  = ypr_update.roll;
                    navx->compass_heading       = ypr_update.compass_heading;
                    navx->last_sensor_timestamp = sensor_timestamp;
                }

                void setNavXPosData(NavXProtocol::NavXPosUpdate& navx_update, long sensor_timestamp)
                {
                    /* Update base IMU class variables */
                    //TODO: add spdlogging here
                    //printf("Setting pitch to: %f\n", NAVX_update.pitch);
                    navx->yaw                    = navx_update.yaw;
                    navx->pitch                  = navx_update.pitch;
                    navx->roll                   = navx_update.roll;
                    navx->compass_heading        = navx_update.compass_heading;
                    navx->yaw_offset_tracker->updateHistory(navx_update.yaw);

                    /* Update NAVX class variables */

                    // 9-axis data
                    navx->fused_heading          = navx_update.fused_heading;

                    // Gravity-corrected linear acceleration(world-frame)
                    navx->world_linear_accel_x   = navx_update.linear_accel_x;
                    navx->world_linear_accel_y   = navx_update.linear_accel_y;
                    navx->world_linear_accel_z   = navx_update.linear_accel_z;

                    // Gyro/Accelerometer Die Temperature
                    navx->mpu_temp_c             = navx_update.mpu_temp;

                    // Barometric Pressure/Altitude
                    navx->altitude               = navx_update.altitude;
                    navx->baro_pressure          = navx_update.barometric_pressure;

                    // Status/Motion Detection
                    navx->is_moving              =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_MOVING) != 0);
                    navx->is_rotating                =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_YAW_STABLE) != 0);
                    navx->altitude_valid             =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0);
                    navx->is_magnetometer_calibrated =
                        ((navx_update.cal_status &
                          NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0);
                    navx->magnetic_disturbance       =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0);

                    navx->quaternionW                = navx_update.quat_w;
                    navx->quaternionX                = navx_update.quat_x;
                    navx->quaternionY                = navx_update.quat_y;
                    navx->quaternionZ                = navx_update.quat_z;

                    navx->last_sensor_timestamp = sensor_timestamp;

                    /* Notify external data arrival subscribers, if any. */
                    for (int i = 0; i < MAX_NUM_CALLBACKS; i++)
                    {
                        ITimestampedDataSubscriber* callback = navx->callbacks[i];
                        if (callback != NULL)
                        {
                            long system_timestamp = (long)(std::time(nullptr) * 1000);
                            callback->timestampedDataReceived(system_timestamp,
                                                              sensor_timestamp,
                                                              navx_update,
                                                              navx->callback_contexts[i]);
                        }
                    }

                    navx->velocity[0]     = navx_update.vel_x;
                    navx->velocity[1]     = navx_update.vel_y;
                    navx->velocity[2]     = navx_update.vel_z;
                    navx->displacement[0] = navx_update.disp_x;
                    navx->displacement[1] = navx_update.disp_y;
                    navx->displacement[2] = navx_update.disp_z;

                    navx->yaw_angle_tracker->nextAngle((navx->getYaw()).to(units::deg));
                    navx->last_sensor_timestamp = sensor_timestamp;
                }

                void setRawData(NavXProtocol::GyroUpdate& raw_data_update, long sensor_timestamp)
                {
                    navx->raw_gyro_x     = raw_data_update.gyro_x;
                    navx->raw_gyro_y     = raw_data_update.gyro_y;
                    navx->raw_gyro_z     = raw_data_update.gyro_z;
                    navx->raw_accel_x    = raw_data_update.accel_x;
                    navx->raw_accel_y    = raw_data_update.accel_y;
                    navx->raw_accel_z    = raw_data_update.accel_z;
                    navx->cal_mag_x      = raw_data_update.mag_x;
                    navx->cal_mag_y      = raw_data_update.mag_y;
                    navx->cal_mag_z      = raw_data_update.mag_z;
                    navx->mpu_temp_c     = raw_data_update.temp_c;
                    navx->last_sensor_timestamp = sensor_timestamp;
                }

                void setNavXData(NavXProtocol::NavXUpdate& navx_update, long sensor_timestamp)
                {
                    /* Update base IMU class variables */

                    navx->yaw                    = navx_update.yaw;
                    navx->pitch                  = navx_update.pitch;
                    navx->roll                   = navx_update.roll;
                    navx->compass_heading        = navx_update.compass_heading;
                    navx->yaw_offset_tracker->updateHistory(navx_update.yaw);

                    /* Update NAVX class variables */

                    // 9-axis data
                    navx->fused_heading          = navx_update.fused_heading;

                    // Gravity-corrected linear acceleration(world-frame)
                    navx->world_linear_accel_x   = navx_update.linear_accel_x;
                    navx->world_linear_accel_y   = navx_update.linear_accel_y;
                    navx->world_linear_accel_z   = navx_update.linear_accel_z;

                    // Gyro/Accelerometer Die Temperature
                    navx->mpu_temp_c             = navx_update.mpu_temp;

                    // Barometric Pressure/Altitude
                    navx->altitude               = navx_update.altitude;
                    navx->baro_pressure          = navx_update.barometric_pressure;

                    // Magnetometer Data
                    navx->cal_mag_x              = navx_update.cal_mag_x;
                    navx->cal_mag_y              = navx_update.cal_mag_y;
                    navx->cal_mag_z              = navx_update.cal_mag_z;

                    // Status/Motion Detection
                    navx->is_moving              =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_MOVING) != 0);

                    navx->is_rotating                =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_YAW_STABLE) != 0);

                    navx->altitude_valid             =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0);

                    navx->is_magnetometer_calibrated =
                        ((navx_update.cal_status &
                          NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0);

                    navx->magnetic_disturbance       =
                        ((navx_update.sensor_status &
                          NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0);

                    navx->quaternionW                = navx_update.quat_w;
                    navx->quaternionX                = navx_update.quat_x;
                    navx->quaternionY                = navx_update.quat_y;
                    navx->quaternionZ                = navx_update.quat_z;

                    navx->last_sensor_timestamp = sensor_timestamp;

                    /* Notify external data arrival subscribers, if any. */
                    for (int i = 0; i < MAX_NUM_CALLBACKS; i++)
                    {
                        ITimestampedDataSubscriber* callback = navx->callbacks[i];
                        if (callback != NULL)
                        {
                            long system_timestamp = (long)(std::time(nullptr) * 1000);
                            callback->timestampedDataReceived(system_timestamp,
                                                              sensor_timestamp,
                                                              navx_update,
                                                              navx->callback_contexts[i]);
                        }
                    }

                    navx->updateDisplacement(navx->world_linear_accel_x,
                                             navx->world_linear_accel_y,
                                             navx->update_rate_hz,
                                             navx->is_moving);

                    navx->yaw_angle_tracker->nextAngle((navx->getYaw())());
                }

                void setBoardID(NavXProtocol::BoardID& board_id)
                {
                    navx->board_type = board_id.type;
                    navx->hw_rev = board_id.hw_rev;
                    navx->fw_ver_major = board_id.fw_ver_major;
                    navx->fw_ver_minor = board_id.fw_ver_minor;
                }

                void setBoardState(IIOCompleteNotification::BoardState& board_state)
                {
                    navx->update_rate_hz = board_state.update_rate_hz;
                    navx->accel_fsr_g = board_state.accel_fsr_g;
                    navx->gyro_fsr_dps = board_state.gyro_fsr_dps;
                    navx->capability_flags = board_state.capability_flags;
                    navx->op_status = board_state.op_status;
                    navx->sensor_status = board_state.sensor_status;
                    navx->cal_status = board_state.cal_status;
                    navx->selftest_status = board_state.selftest_status;
                }

                /***********************************************************/
                /* IBoardCapabilities Interface Implementation        */
                /***********************************************************/
                bool isOmniMountSupported()
                {
                    return ((navx->capability_flags & NAVX_CAPABILITY_FLAG_OMNIMOUNT) != 0);
                }

                bool isBoardYawResetSupported()
                {
                    return ((navx->capability_flags & NAVX_CAPABILITY_FLAG_YAW_RESET) != 0);
                }

                bool isDisplacementSupported()
                {
                    return ((navx->capability_flags & NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0);
                }

                bool isNavXPosTimestampSupported()
                {
                    return ((navx->capability_flags & NAVX_CAPABILITY_FLAG_NAVXPOS_TS) != 0);
                }
            };

            NavX::NavX(std::string serial_port_id, NavX::serialDataType data_type, uint8_t update_rate_hz)
                :Subsystem("navx")
            {
                serialInit(serial_port_id, data_type, update_rate_hz);
            }

            NavX::NavX(std::string serial_port_id)
                :Subsystem("navx")
            {
                serialInit(serial_port_id, serialDataType::kProcessedData, NAVX_DEFAULT_UPDATE_RATE_HZ);
                misc::Logger::getInstance()->debug(fmt::format("navx constructed, device: {}"
                , serial_port_id));
            }

            units::Angle NavX::getPitch()
            {
                return static_cast<float>(pitch) * units::deg;
            }

            units::Angle NavX::getRoll()
            {
                return static_cast<float>(roll) * units::degrees;
            }

            units::Angle NavX::getYaw()
            {
                if (navx_internal->isBoardYawResetSupported())
                {
                    return static_cast<float>((this->yaw)) * units::degrees;
                }
                else
                {
                    return ((float) yaw_offset_tracker->applyOffset(this->yaw)) * units::degrees;
                }
            }

            units::Angle NavX::getCompassHeading()
            {
                return static_cast<float>(compass_heading) * units::degrees;
            }

            void NavX::zeroYaw()
            {
                if (navx_internal->isBoardYawResetSupported())
                {
                    io->zeroYaw();
                    //yaw_angle_tracker->reset();
                }
                else
                {
                    //yaw_angle_tracker->reset();
                    yaw_offset_tracker->setOffset();
                }
            }

            bool NavX::isCalibrating()
            {
                return !((cal_status &
                          NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) ==
                         NAVX_CAL_STATUS_IMU_CAL_COMPLETE);
            }

            bool NavX::isConnected()
            {
                return io->isConnected();
            }

            double NavX::getByteCount()
            {
                return io->getByteCount();
            }

            double NavX::getUpdateCount()
            {
                return io->getUpdateCount();
            }

            long NavX::getLastSensorTimestamp()
            {
                return this->last_sensor_timestamp;
            }

            units::Acceleration NavX::getWorldLinearAccelX()
            {
                return static_cast<double>(this->world_linear_accel_x) * units::AccelerationOfGravity;
            }

            units::Acceleration NavX::getWorldLinearAccelY()
            {
                return static_cast<double>(this->world_linear_accel_y) * units::AccelerationOfGravity;
            }

            units::Acceleration NavX::getWorldLinearAccelZ()
            {
                return static_cast<double>(this->world_linear_accel_z) * units::AccelerationOfGravity;
            }

            bool NavX::isMoving()
            {
                return is_moving;
            }

            bool NavX::isRotating()
            {
                return is_rotating;
            }

            units::Pressure NavX::getBarometricPressure()
            {
                return static_cast<float>(baro_pressure) * units::millibar;
            }

            units::Distance NavX::getAltitude()
            {
                return static_cast<float>(altitude) * units::m;
            }

            bool NavX::isAltitudeValid()
            {
                return this->altitude_valid;
            }

            units::Angle NavX::getFusedHeading()
            {
                return static_cast<float>(fused_heading) * units::degrees;
            }

            bool NavX::isMagneticDisturbance()
            {
                return magnetic_disturbance;
            }

            bool NavX::isMagnetometerCalibrated()
            {
                return is_magnetometer_calibrated;
            }

            float NavX::getQuaternionW()
            {
                return quaternionW;
            }

            float NavX::getQuaternionX()
            {
                return quaternionX;
            }

            float NavX::getQuaternionY()
            {
                return quaternionY;
            }

            float NavX::getQuaternionZ()
            {
                return quaternionZ;
            }

            void NavX::resetDisplacement()
            {
                if (navx_internal->isDisplacementSupported())
                {
                    io->zeroDisplacement();
                }
                else
                {
                    integrator->resetDisplacement();
                }
            }

            void NavX::updateDisplacement(float accel_x_g, float accel_y_g,
                                          int update_rate_hz, bool is_moving)
            {
                integrator->updateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
            }

            units::Velocity NavX::getVelocityX()
            {
                return units::m / units::s * (navx_internal->isDisplacementSupported() ? velocity[0] : integrator->getVelocityX());
            }

            units::Velocity NavX::getVelocityY()
            {
                return units::m / units::s * (navx_internal->isDisplacementSupported() ? velocity[1] : integrator->getVelocityY());
            }

            units::Velocity NavX::getVelocityZ()
            {
                return units::m / units::s * (navx_internal->isDisplacementSupported() ? velocity[2] : 0.f);
            }

            units::Distance NavX::getDisplacementX()
            {
                return units::m * (navx_internal->isDisplacementSupported() ? displacement[0] : integrator->getVelocityX());
            }

            units::Distance NavX::getDisplacementY()
            {
                return units::m * (navx_internal->isDisplacementSupported() ? displacement[1] : integrator->getVelocityY());
            }

            units::Distance NavX::getDisplacementZ()
            {
                return units::m * (navx_internal->isDisplacementSupported() ? displacement[2] : 0.f);
            }

            void NavX::serialInit(std::string serial_port_id, NavX::serialDataType data_type, uint8_t update_rate_hz)
            {
                commonInit(update_rate_hz);
                bool processed_data = (data_type == serialDataType::kProcessedData);
                io = new SerialIO(serial_port_id, update_rate_hz, processed_data, navx_internal, navx_internal);
                ::pthread_t trd;
                ::pthread_create(&trd, NULL, NavX::threadFunc, io);
            }

            void NavX::commonInit(uint8_t update_rate_hz)
            {

                navx_internal = new NavXInternal(this);
                this->update_rate_hz = update_rate_hz;

                /* Processed Data */

                yaw_offset_tracker = new OffsetTracker(YAW_HISTORY_LENGTH);
                integrator = new InertialDataIntegrator();
                yaw_angle_tracker = new ContinuousAngleTracker();

                yaw = 0.0f;
                pitch = 0.0f;
                roll =  0.0f;
                compass_heading = 0.0f;
                world_linear_accel_x =  0.0f;
                world_linear_accel_y =  0.0f;
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
                quaternionW = 0.0f;
                quaternionX = 0.0f;
                quaternionY = 0.0f;
                quaternionZ = 0.0f;

                /* Integrated Data */

                for (int i = 0; i < 3; i++)
                {
                    velocity[i] = 0.0f;
                    displacement[i] = 0.0f;
                }

                /* Raw Data */
                raw_gyro_x = 0.0f;
                raw_gyro_y = 0.0f;
                raw_gyro_z = 0.0f;
                raw_accel_x = 0.0f;
                raw_accel_y = 0.0f;
                raw_accel_z = 0.0f;
                cal_mag_x = 0.0f;
                cal_mag_y = 0.0f;
                cal_mag_z = 0.0f;

                /* Configuration/Status */
                update_rate_hz = 0;
                accel_fsr_g = DEFAULT_ACCEL_FSR_G;
                gyro_fsr_dps = DEFAULT_GYRO_FSR_DPS;
                capability_flags = 0;
                op_status = 0;
                sensor_status = 0;
                cal_status = 0;
                selftest_status = 0;
                /* Board ID */
                board_type = 0;
                hw_rev = 0;
                fw_ver_major = 0;
                fw_ver_minor = 0;
                last_sensor_timestamp = 0;
                last_update_time = 0;

                io = 0;

                for (int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    callbacks[i] = NULL;
                    callback_contexts[i] = NULL;
                }
            }

            units::Angle NavX::getAngle()
            {
                return yaw_angle_tracker->getAngle() * units::deg;
            }

            units::AngularVelocity NavX::getRate()
            {
                return yaw_angle_tracker->getRate() * units::degrees / units::s;
            }

            void NavX::reset()
            {
                zeroYaw();
            }

            static const float DEV_UNITS_MAX = 32768.0f;

            float NavX::getRawGyroX()
            {
                return this->raw_gyro_x / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
            }

            float NavX::getRawGyroY()
            {
                return this->raw_gyro_y / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
            }

            float NavX::getRawGyroZ()
            {
                return this->raw_gyro_z / (DEV_UNITS_MAX / (float)gyro_fsr_dps);
            }

            float NavX::getRawAccelX()
            {
                return this->raw_accel_x / (DEV_UNITS_MAX / (float)accel_fsr_g);
            }

            float NavX::getRawAccelY()
            {
                return this->raw_accel_y / (DEV_UNITS_MAX / (float)accel_fsr_g);
            }


            float NavX::getRawAccelZ()
            {
                return this->raw_accel_z / (DEV_UNITS_MAX / (float)accel_fsr_g);
            }

            static const float UTESLA_PER_DEV_UNIT = 0.15f;

            float NavX::getRawMagX()
            {
                return this->cal_mag_x / UTESLA_PER_DEV_UNIT;
            }

            float NavX::getRawMagY()
            {
                return this->cal_mag_y / UTESLA_PER_DEV_UNIT;
            }

            float NavX::getRawMagZ()
            {
                return this->cal_mag_z / UTESLA_PER_DEV_UNIT;
            }

            units::Temperature NavX::getTempC()
            {
                return static_cast<float>(this->mpu_temp_c) * units::degC;
            }

            NavX::BoardYawAxis NavX::getBoardYawAxis()
            {
                BoardYawAxis yaw_axis;
                short yaw_axis_info = (short)(capability_flags >> 3);
                yaw_axis_info &= 7;
                if (yaw_axis_info == OMNIMOUNT_DEFAULT)
                {
                    yaw_axis.up = true;
                    yaw_axis.board_axis = boardAxis::kBoardAxisZ;
                }
                else
                {
                    yaw_axis.up = ((yaw_axis_info & 0x01) != 0);
                    yaw_axis_info >>= 1;
                    switch (yaw_axis_info)
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

            std::string NavX::getFirmwareVersion()
            {
                std::ostringstream os;
                os << (int)fw_ver_major << "." << (int)fw_ver_minor;
                std::string fw_version = os.str();
                return fw_version;
            }

            void* NavX::threadFunc(void* threadarg)
            {
                IIOProvider* io_provider = (IIOProvider*)threadarg;
                io_provider->run();
                return NULL;
            }

            bool NavX::registerCallback(ITimestampedDataSubscriber* callback, void* callback_context)
            {
                bool registered = false;
                for (int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    if (callbacks[i] == NULL)
                    {
                        callbacks[i] = callback;
                        callback_contexts[i] = callback_context;
                        registered = true;
                        break;
                    }
                }
                return registered;
            }

            bool NavX::deregisterCallback(ITimestampedDataSubscriber* callback)
            {
                bool deregistered = false;
                for (int i = 0; i < MAX_NUM_CALLBACKS; i++)
                {
                    if (callbacks[i] == callback)
                    {
                        callbacks[i] = NULL;
                        deregistered = true;
                        break;
                    }
                }
                return deregistered;
            }

            int NavX::getActualUpdateRate()
            {
                uint8_t actual_update_rate = getActualUpdateRateInternal(getRequestedUpdateRate());
                return (int)actual_update_rate;
            }

            uint8_t NavX::getActualUpdateRateInternal(uint8_t update_rate)
            {
#define NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ 200
                int integer_update_rate = (int)update_rate;
                int realized_update_rate = NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ /
                                           (NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ / integer_update_rate);
                return (uint8_t)realized_update_rate;
            }

            int NavX::getRequestedUpdateRate()
            {
                return (int)update_rate_hz;
            }

            void NavX::close()
            {
                io->stop();
            }

            void NavX::stop()
            {
                close();
            }

            bool NavX::diagnostic()
            {
                //todo
                return 0;
            }

            std::shared_ptr<NavX> NavX::makeNavX(const nlohmann::json& config)
            {
                std::string device = config["device"];
                return std::make_shared<NavX>(device);
            }


        }
    }
}
