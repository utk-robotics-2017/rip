
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

            /**
             * The AHRS class provides an interface to AHRS capabilities
             * of the KauaiLabs navX Robotics Navigation Sensor via SPI, I2C and
             * Serial(TTL UART and USB) communications interfaces on RIP.
             *
             * The AHRS class enables access to basic connectivity and state information,
             * as well as key 6-axis and 9-axis orientation information(yaw, pitch, roll,
             * compass heading, fused(9-axis) heading and magnetic disturbance detection.
             *
             * Additionally, the ARHS class also provides access to extended information
             * including linear acceleration, motion detection, rotation detection and sensor
             * temperature.
             *
             * If used with the navX Aero, the AHRS class also provides access to
             * altitude, barometric pressure and pressure sensor temperature data
             * @author Scott & UTK IEEE Robotics.
             */
             using namespace rip;
            class AHRS
            {
            public:

                enum boardAxis
                {
                    kBoardAxisX = 0,
                    kBoardAxisY = 1,
                    kBoardAxisZ = 2,
                };

                struct BoardYawAxis
                {
                    /* Identifies one of the board axes */
                    boardAxis board_axis;
                    /* true if axis is pointing up(with respect to gravity); false if pointing down. */
                    bool up;
                };

                enum serialDataType
                {
                /**
                 *(default):  6 and 9-axis processed data
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
                volatile float      quaternionW;
                volatile float      quaternionX;
                volatile float      quaternionY;
                volatile float      quaternionZ;

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

                 /**
                  * Constructs the AHRS class using serial communication and the default update rate,
                  * and returning processed(rather than raw) data.
                  *<p>
                  * This constructor should be used if communicating via either
                  * TTL UART or USB Serial interface.
                  *<p>
                  * @param serial_port_id SerialPort to use
                  */
                AHRS(std::string serial_port_id);
                /**
                 * Constructs the AHRS class using serial communication, overriding the
                 * default update rate with a custom rate which may be from 4 to 200,
                 * representing the number of updates per second sent by the sensor.
                 *<p>
                 * This constructor should be used if communicating via either
                 * TTL UART or USB Serial interface.
                 *<p>
                 * Note that the serial interfaces can communicate either
                 * processed data, or raw data, but not both simultaneously.
                 * If simultaneous processed and raw data are needed, use
                 * one of the register-based interfaces(SPI or I2C).
                 *<p>
                 * Note that increasing the update rate may increase the
                 * CPU utilization.
                 *<p>
                 * @param serial_port_id SerialPort to use
                 * @param data_type either kProcessedData or kRawData
                 * @param update_rate_hz Custom Update Rate(Hz)
                 */
                AHRS(std::string serial_port_id, AHRS::serialDataType data_type, uint8_t update_rate_hz);

                /**
                 * Returns the current pitch value(in degrees, from -180 to 180)
                 * reported by the sensor.  Pitch is a measure of rotation around
                 * the X Axis.
                 * @return The current pitch value in degrees(-180 to 180).
                 */
                units::Angle getPitch();
                /**
                 * Returns the current roll value(in degrees, from -180 to 180)
                 * reported by the sensor.  Roll is a measure of rotation around
                 * the X Axis.
                 * @return The current roll value in degrees(-180 to 180).
                 */
                units::Angle getRoll();
                /**
                 * Returns the current yaw value(in degrees, from -180 to 180)
                 * reported by the sensor.  Yaw is a measure of rotation around
                 * the Z Axis(which is perpendicular to the earth).
                 *<p>
                 * Note that the returned yaw value will be offset by a user-specified
                 * offset value; this user-specified offset value is set by
                 * invoking the zeroYaw() method.
                 * @return The current yaw value in degrees(-180 to 180).
                 */
                units::Angle getYaw();
                /**
                 * Returns the current tilt-compensated compass heading
                 * value(in degrees, from 0 to 360) reported by the sensor.
                 *<p>
                 * Note that this value is sensed by a magnetometer,
                 * which can be affected by nearby magnetic fields(e.g., the
                 * magnetic fields generated by nearby motors).
                 *<p>
                 * Before using this value, ensure that(a) the magnetometer
                 * has been calibrated and(b) that a magnetic disturbance is
                 * not taking place at the instant when the compass heading
                 * was generated.
                 * @return The current tilt-compensated compass heading, in degrees(0-360).
                 */
                units::Angle getCompassHeading();
                /**
                 * Sets the user-specified yaw offset to the current
                 * yaw value reported by the sensor.
                 *<p>
                 * This user-specified yaw offset is automatically
                 * subtracted from subsequent yaw values reported by
                 * the getYaw() method.
                 */
                void   zeroYaw();
                /**
                 * Returns true if the sensor is currently performing automatic
                 * gyro/accelerometer calibration.  Automatic calibration occurs
                 * when the sensor is initially powered on, during which time the
                 * sensor should be held still, with the Z-axis pointing up
                 *(perpendicular to the earth).
                 *<p>
                 * NOTE:  During this automatic calibration, the yaw, pitch and roll
                 * values returned may not be accurate.
                 *<p>
                 * Once calibration is complete, the sensor will automatically remove
                 * an internal yaw offset value from all reported values.
                 *<p>
                 * @return Returns true if the sensor is currently automatically
                 * calibrating the gyro and accelerometer sensors.
                 */
                bool   isCalibrating();
                /**
                 * Indicates whether the sensor is currently connected
                 * to the host computer.  A connection is considered established
                 * whenever communication with the sensor has occurred recently.
                 *<p>
                 * @return Returns true if a valid update has been recently received
                 * from the sensor.
                 */
                bool   isConnected();
                /**
                 * Returns the count in bytes of data received from the
                 * sensor.  This could can be useful for diagnosing
                 * connectivity issues.
                 *<p>
                 * If the byte count is increasing, but the update count
                 *(see getUpdateCount()) is not, this indicates a software
                 * misconfiguration.
                 * @return The number of bytes received from the sensor.
                 */
                double getByteCount();
                /**
                 * Returns the count of valid updates which have
                 * been received from the sensor.  This count should increase
                 * at the same rate indicated by the configured update rate.
                 * @return The number of valid updates received from the sensor.
                 */
                double getUpdateCount();
                /**
                 * Returns the sensor timestamp corresponding to the
                 * last sample retrieved from the sensor.  Note that this
                 * sensor timestamp is only provided when the Register-based
                 * IO methods(SPI, I2C) are used; sensor timestamps are not
                 * provided when Serial-based IO methods(TTL UART, USB)
                 * are used.
                 * @return The sensor timestamp corresponding to the current AHRS sensor data.
                 */
                long   getLastSensorTimestamp();
                /**
                 * Returns the current linear acceleration in the X-axis(in G).
                 *<p>
                 * World linear acceleration refers to raw acceleration data, which
                 * has had the gravity component removed, and which has been rotated to
                 * the same reference frame as the current yaw value.  The resulting
                 * value represents the current acceleration in the x-axis of the
                 * body(e.g., the robot) on which the sensor is mounted.
                 *<p>
                 * @return Current world linear acceleration in the X-axis(in G).
                 */
                units::Acceleration getWorldLinearAccelX();
                /**
                 * Returns the current linear acceleration in the Y-axis(in G).
                 *<p>
                 * World linear acceleration refers to raw acceleration data, which
                 * has had the gravity component removed, and which has been rotated to
                 * the same reference frame as the current yaw value.  The resulting
                 * value represents the current acceleration in the Y-axis of the
                 * body(e.g., the robot) on which the sensor is mounted.
                 *<p>
                 * @return Current world linear acceleration in the Y-axis(in G).
                 */
                units::Acceleration getWorldLinearAccelY();
                /**
                 * Returns the current linear acceleration in the Z-axis(in G).
                 *<p>
                 * World linear acceleration refers to raw acceleration data, which
                 * has had the gravity component removed, and which has been rotated to
                 * the same reference frame as the current yaw value.  The resulting
                 * value represents the current acceleration in the Z-axis of the
                 * body(e.g., the robot) on which the sensor is mounted.
                 *<p>
                 * @return Current world linear acceleration in the Z-axis(in G).
                 */
                units::Acceleration getWorldLinearAccelZ();
                /**
                 * Indicates if the sensor is currently detecting motion,
                 * based upon the X and Y-axis world linear acceleration values.
                 * If the sum of the absolute values of the X and Y axis exceed
                 * a "motion threshold", the motion state is indicated.
                 *<p>
                 * @return Returns true if the sensor is currently detecting motion.
                 */
                bool   isMoving();
                /**
                 * Indicates if the sensor is currently detecting yaw rotation,
                 * based upon whether the change in yaw over the last second
                 * exceeds the "Rotation Threshold."
                 *<p>
                 * Yaw Rotation can occur either when the sensor is rotating, or
                 * when the sensor is not rotating AND the current gyro calibration
                 * is insufficiently calibrated to yield the standard yaw drift rate.
                 *<p>
                 * @return Returns true if the sensor is currently detecting motion.
                 */
                bool   isRotating();
                /**
                 * Returns the current barometric pressure, based upon calibrated readings
                 * from the onboard pressure sensor.  This value is in units of millibar.
                 *<p>
                 * NOTE:  This value is only valid for a navX Aero.  To determine
                 * whether this value is valid, see isAltitudeValid().
                 * @return Returns current barometric pressure(navX Aero only).
                 */
                units::Pressure getBarometricPressure();
                /**
                 * Returns the current altitude, based upon calibrated readings
                 * from a barometric pressure sensor, and the currently-configured
                 * sea-level barometric pressure [navX Aero only].  This value is in units of meters.
                 *<p>
                 * NOTE:  This value is only valid sensors including a pressure
                 * sensor.  To determine whether this value is valid, see
                 * isAltitudeValid().
                 *<p>
                 * @return Returns current altitude in meters(as long as the sensor includes
                 * an installed on-board pressure sensor).
                 */
                units::Distance getAltitude();
                /**
                 * Indicates whether the current altitude(and barometric pressure) data is
                 * valid. This value will only be true for a sensor with an onboard
                 * pressure sensor installed.
                 *<p>
                 * If this value is false for a board with an installed pressure sensor,
                 * this indicates a malfunction of the onboard pressure sensor.
                 *<p>
                 * @return Returns true if a working pressure sensor is installed.
                 */
                bool   isAltitudeValid();
                /**
                 * Returns the "fused"(9-axis) heading.
                 *<p>
                 * The 9-axis heading is the fusion of the yaw angle, the tilt-corrected
                 * compass heading, and magnetic disturbance detection.  Note that the
                 * magnetometer calibration procedure is required in order to
                 * achieve valid 9-axis headings.
                 *<p>
                 * The 9-axis Heading represents the sensor's best estimate of current heading,
                 * based upon the last known valid Compass Angle, and updated by the change in the
                 * Yaw Angle since the last known valid Compass Angle.  The last known valid Compass
                 * Angle is updated whenever a Calibrated Compass Angle is read and the sensor
                 * has recently rotated less than the Compass Noise Bandwidth(~2 degrees).
                 * @return Fused Heading in Degrees(range 0-360)
                 */
                units::Angle  getFusedHeading();
                /**
                 * Indicates whether the current magnetic field strength diverges from the
                 * calibrated value for the earth's magnetic field by more than the currently-
                 * configured Magnetic Disturbance Ratio.
                 *<p>
                 * This function will always return false if the sensor's magnetometer has
                 * not yet been calibrated; see isMagnetometerCalibrated().
                 * @return true if a magnetic disturbance is detected(or the magnetometer is uncalibrated).
                 */
                bool   isMagneticDisturbance();
                /**
                 * Indicates whether the magnetometer has been calibrated.
                 *<p>
                 * Magnetometer Calibration must be performed by the user.
                 *<p>
                 * Note that if this function does indicate the magnetometer is calibrated,
                 * this does not necessarily mean that the calibration quality is sufficient
                 * to yield valid compass headings.
                 *<p>
                 * @return Returns true if magnetometer calibration has been performed.
                 */
                bool   isMagnetometerCalibrated();
                /* Unit Quaternions */

                /**
                 * Returns the imaginary portion(W) of the Orientation Quaternion which
                 * fully describes the current sensor orientation with respect to the
                 * reference angle defined as the angle at which the yaw was last "zeroed".
                 *<p>
                 * Each quaternion value(W,X,Y,Z) is expressed as a value ranging from -2
                 * to 2.  This total range(4) can be associated with a unit circle, since
                 * each circle is comprised of 4 PI Radians.
                 * <p>
                 * For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
                 * @return Returns the imaginary portion(W) of the quaternion.
                 */
                float  getQuaternionW();
                /**
                 * Returns the real portion(X axis) of the Orientation Quaternion which
                 * fully describes the current sensor orientation with respect to the
                 * reference angle defined as the angle at which the yaw was last "zeroed".
                 * <p>
                 * Each quaternion value(W,X,Y,Z) is expressed as a value ranging from -2
                 * to 2.  This total range(4) can be associated with a unit circle, since
                 * each circle is comprised of 4 PI Radians.
                 * <p>
                 * For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
                 * @return Returns the real portion(X) of the quaternion.
                 */
                float  getQuaternionX();
                /**
                 * Returns the real portion(X axis) of the Orientation Quaternion which
                 * fully describes the current sensor orientation with respect to the
                 * reference angle defined as the angle at which the yaw was last "zeroed".
                 *
                 * Each quaternion value(W,X,Y,Z) is expressed as a value ranging from -2
                 * to 2.  This total range(4) can be associated with a unit circle, since
                 * each circle is comprised of 4 PI Radians.
                 *
                 * For more information on Quaternions and their use, please see:
                 *
                 *   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
                 *
                 * @return Returns the real portion(X) of the quaternion.
                 */
                float  getQuaternionY();
                /**
                 * Returns the real portion(X axis) of the Orientation Quaternion which
                 * fully describes the current sensor orientation with respect to the
                 * reference angle defined as the angle at which the yaw was last "zeroed".
                 *
                 * Each quaternion value(W,X,Y,Z) is expressed as a value ranging from -2
                 * to 2.  This total range(4) can be associated with a unit circle, since
                 * each circle is comprised of 4 PI Radians.
                 *
                 * For more information on Quaternions and their use, please see:
                 *
                 *   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
                 *
                 * @return Returns the real portion(X) of the quaternion.
                 */
                float  getQuaternionZ();
                /**
                 * Zeros the displacement integration variables.   Invoke this at the moment when
                 * integration begins.
                 */
                void   resetDisplacement();
                /**
                 * Each time new linear acceleration samples are received, this function should be invoked.
                 * This function transforms acceleration in G to meters/sec^2, then converts this value to
                 * Velocity in meters/sec(based upon velocity in the previous sample).  Finally, this value
                 * is converted to displacement in meters, and integrated.
                 * @return none.
                 */
                void   updateDisplacement(float accel_x_g, float accel_y_g,
                                           int update_rate_hz, bool is_moving);
               /**
                * Returns the velocity(in meters/sec) of the X axis [Experimental].
                *
                * NOTE:  This feature is experimental.  Velocity measures rely on integration
                * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                * resulting velocities are not known to be very accurate.
                * @return Current Velocity(in meters/squared).
                */
                units::Velocity getVelocityX();
                /**
                 * Returns the velocity(in meters/sec) of the Y axis [Experimental].
                 *
                 * NOTE:  This feature is experimental.  Velocity measures rely on integration
                 * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                 * resulting velocities are not known to be very accurate.
                 * @return Current Velocity(in meters/squared).
                 */
                units::Velocity getVelocityY();
                /**
                 * Returns the velocity(in meters/sec) of the Z axis [Experimental].
                 *
                 * NOTE:  This feature is experimental.  Velocity measures rely on integration
                 * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                 * resulting velocities are not known to be very accurate.
                 * @return Current Velocity(in meters/squared).
                 */
                units::Velocity getVelocityZ();
                /**
                 * Returns the displacement(in meters) of the X axis since resetDisplacement()
                 * was last invoked [Experimental].
                 *
                 * NOTE:  This feature is experimental.  Displacement measures rely on double-integration
                 * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                 * resulting displacement are not known to be very accurate, and the amount of error
                 * increases quickly as time progresses.
                 * @return Displacement since last reset(in meters).
                 */
                units::Distance getDisplacementX();
                /**
                 * Returns the displacement(in meters) of the Y axis since resetDisplacement()
                 * was last invoked [Experimental].
                 *
                 * NOTE:  This feature is experimental.  Displacement measures rely on double-integration
                 * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                 * resulting displacement are not known to be very accurate, and the amount of error
                 * increases quickly as time progresses.
                 * @return Displacement since last reset(in meters).
                 */
                units::Distance getDisplacementY();
                /**
                 * Returns the displacement(in meters) of the Z axis since resetDisplacement()
                 * was last invoked [Experimental].
                 *
                 * NOTE:  This feature is experimental.  Displacement measures rely on double-integration
                 * of acceleration values from MEMS accelerometers which yield "noisy" values.  The
                 * resulting displacement are not known to be very accurate, and the amount of error
                 * increases quickly as time progresses.
                 * @return Displacement since last reset(in meters).
                 */
                units::Distance getDisplacementZ();
                /**
                 * Returns the total accumulated yaw angle(Z Axis, in degrees)
                 * reported by the sensor.
                 *<p>
                 * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees.
                 * This ensures that algorithms that wouldn't want to see a discontinuity
                 * in the gyro output as it sweeps past 0 on the second time around.
                 *<p>
                 * Note that the returned yaw value will be offset by a user-specified
                 * offset value; this user-specified offset value is set by
                 * invoking the zeroYaw() method.
                 *<p>
                 * @return The current total accumulated yaw angle(Z axis) of the robot
                 * in degrees. This heading is based on integration of the returned rate
                 * from the Z-axis(yaw) gyro.
                 */
                units::Angle getAngle();
                /**
                 * Return the rate of rotation of the yaw(Z-axis) gyro, in degrees per second.
                 *<p>
                 * The rate is based on the most recent reading of the yaw gyro angle.
                 *<p>
                 * @return The current rate of change in yaw angle(in degrees per second)
                 */

                units::AngularVelocity getRate();
                /**
                 * Reset the Yaw gyro.
                 *<p>
                 * Resets the Gyro Z(Yaw) axis to a heading of zero. This can be used if
                 * there is significant drift in the gyro and it needs to be recalibrated
                 * after it has been running.
                 */
                void   reset();
                /**
                 * Returns the current raw(unprocessed) X-axis gyro rotation rate(in degrees/sec).  NOTE:  this
                 * value is un-processed, and should only be accessed by advanced users.
                 * Typically, rotation about the X Axis is referred to as "Pitch".  Calibrated
                 * and Integrated Pitch data is accessible via the {@link #GetPitch()} method.
                 *<p>
                 * @return Returns the current rotation rate(in degrees/sec).
                 */
                float  getRawGyroX();
                /**
                 * Returns the current raw(unprocessed) Y-axis gyro rotation rate(in degrees/sec).  NOTE:  this
                 * value is un-processed, and should only be accessed by advanced users.
                 * Typically, rotation about the T Axis is referred to as "Roll".  Calibrated
                 * and Integrated Pitch data is accessible via the {@link #GetRoll()} method.
                 *<p>
                 * @return Returns the current rotation rate(in degrees/sec).
                 */
                float  getRawGyroY();
                /**
                 * Returns the current raw(unprocessed) Z-axis gyro rotation rate(in degrees/sec).  NOTE:  this
                 * value is un-processed, and should only be accessed by advanced users.
                 * Typically, rotation about the T Axis is referred to as "Yaw".  Calibrated
                 * and Integrated Pitch data is accessible via the {@link #GetYaw()} method.
                 *<p>
                 * @return Returns the current rotation rate(in degrees/sec).
                 */
                float  getRawGyroZ();
                /**
                 * Returns the current raw(unprocessed) X-axis acceleration rate(in G).  NOTE:  this
                 * value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not had acceleration due to gravity removed from it, and has not been rotated to
                 * the world reference frame.  Gravity-corrected, world reference frame-corrected
                 * X axis acceleration data is accessible via the {@link #GetWorldLinearAccelX()} method.
                 *<p>
                 * @return Returns the current acceleration rate(in G).
                 */
                float  getRawAccelX();
                /**
                 * Returns the current raw(unprocessed) Y-axis acceleration rate(in G).  NOTE:  this
                 * value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not had acceleration due to gravity removed from it, and has not been rotated to
                 * the world reference frame.  Gravity-corrected, world reference frame-corrected
                 * Y axis acceleration data is accessible via the {@link #GetWorldLinearAccelY()} method.
                 *<p>
                 * @return Returns the current acceleration rate(in G).
                 */
                float  getRawAccelY();
                /**
                 * Returns the current raw(unprocessed) Z-axis acceleration rate(in G).  NOTE:  this
                 * value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not had acceleration due to gravity removed from it, and has not been rotated to
                 * the world reference frame.  Gravity-corrected, world reference frame-corrected
                 * Z axis acceleration data is accessible via the {@link #GetWorldLinearAccelZ()} method.
                 *<p>
                 * @return Returns the current acceleration rate(in G).
                 */
                float  getRawAccelZ();
                /**
                 * Returns the current raw(unprocessed) X-axis magnetometer reading(in uTesla).  NOTE:
                 * this value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not been tilt-corrected, and has not been combined with the other magnetometer axis
                 * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
                 * via the {@link #GetCompassHeading()} method.
                 *<p>
                 * @return Returns the mag field strength(in uTesla).
                 */
                float  getRawMagX();
                /**
                 * Returns the current raw(unprocessed) Y-axis magnetometer reading(in uTesla).  NOTE:
                 * this value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not been tilt-corrected, and has not been combined with the other magnetometer axis
                 * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
                 * via the {@link #GetCompassHeading()} method.
                 *<p>
                 * @return Returns the mag field strength(in uTesla).
                 */
                float  getRawMagY();
                /**
                 * Returns the current raw(unprocessed) Z-axis magnetometer reading(in uTesla).  NOTE:
                 * this value is unprocessed, and should only be accessed by advanced users.  This raw value
                 * has not been tilt-corrected, and has not been combined with the other magnetometer axis
                 * data to yield a compass heading.  Tilt-corrected compass heading data is accessible
                 * via the {@link #GetCompassHeading()} method.
                 *<p>
                 * @return Returns the mag field strength(in uTesla).
                 */
                float  getRawMagZ();
                /**
                 * Returns the current temperature(in degrees centigrade) reported by
                 * the sensor's gyro/accelerometer circuit.
                 *<p>
                 * This value may be useful in order to perform advanced temperature-
                 * correction of raw gyroscope and accelerometer values.
                 *<p>
                 * @return The current temperature(in degrees centigrade).
                 */
                units::Temperature getTempC();
                /**
                 * Returns information regarding which sensor board axis(X,Y or Z) and
                 * direction(up/down) is currently configured to report Yaw(Z) angle
                 * values.   NOTE:  If the board firmware supports Omnimount, the board yaw
                 * axis/direction are configurable.
                 *<p>
                 * For more information on Omnimount, please see:
                 *<p>
                 * http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/
                 *<p>
                 * @return The currently-configured board yaw axis/direction.
                 */
                AHRS::BoardYawAxis getBoardYawAxis();
                /**
                 * Returns the version number of the firmware currently executing
                 * on the sensor.
                 *<p>
                 * To update the firmware to the latest version, please see:
                 *<p>
                 *   http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/
                 *<p>
                 * @return The firmware version in the format [MajorVersion].[MinorVersion]
                 */
                std::string getFirmwareVersion();
                /**
                 * Registers a callback interface.  This interface
                 * will be called back when new data is available,
                 * based upon a change in the sensor timestamp.
                 *<p>
                 * Note that this callback will occur within the context of the
                 * device IO thread, which is not the same thread context the
                 * caller typically executes in.
                 */
                bool registerCallback(ITimestampedDataSubscriber *callback, void *callback_context);
                /**
                 * Deregisters a previously registered callback interface.
                 *
                 * Be sure to deregister any callback which have been
                 * previously registered, to ensure that the object
                 * implementing the callback interface does not continue
                 * to be accessed when no longer necessary.
                 */
                bool deregisterCallback(ITimestampedDataSubscriber *callback);
                /**
                 * Returns the navX-Model device's currently configured update
                 * rate.  Note that the update rate that can actually be realized
                 * is a value evenly divisible by the navX-Model device's internal
                 * motion processor sample clock(200Hz).  Therefore, the rate that
                 * is returned may be lower than the requested sample rate.
                 *
                 * The actual sample rate is rounded down to the nearest integer
                 * that is divisible by the number of Digital Motion Processor clock
                 * ticks.  For instance, a request for 58 Hertz will result in
                 * an actual rate of 66Hz(200 /(200 / 58), using integer
                 * math.
                 *
                 * @return Returns the current actual update rate in Hz
                 *(cycles per second).
                 */

                int getActualUpdateRate();
                /**
                 * Returns the currently requested update rate.
                 * rate.  Note that not every update rate can actually be realized,
                 * since the actual update rate must be a value evenly divisible by
                 * the navX-Model device's internal motion processor sample clock(200Hz).
                 *
                 * To determine the actual update rate, use the
                 * {@link #getActualUpdateRate()} method.
                 *
                 * @return Returns the requested update rate in Hz
                 *(cycles per second).
                 */

                int getRequestedUpdateRate();

                void close();

            private:
                void serialInit(std::string serial_port_id, AHRS::serialDataType data_type, uint8_t update_rate_hz);
                void commonInit(uint8_t update_rate_hz);
                static void *threadFunc(void *threadarg);

                uint8_t getActualUpdateRateInternal(uint8_t update_rate);
            };
        } // navx
    } // navigation
} // rip
#endif /* SRC_AHRS_H_ */
