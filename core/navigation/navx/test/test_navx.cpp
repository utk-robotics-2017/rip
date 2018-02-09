#include "mock_navx.hpp"
#include <exceptions.hpp>
#include <memory>
#include <gtest/gtest.h>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace tests
            {
                TEST(AHRS, SerialConstructor1)
                {
                    // do me
                }
                TEST(AHRS, SerialConstructor2)
                {
                    //
                }
                TEST(AHRS, zeroYaw)
                {

                }

                TEST(AHRS, LinearAccel)
                {

                }
                TEST(AHRS, Quaternion)
                {

                }

                /* GETS */
                TEST(AHRS, getCompassHeading)
                {

                }
                TEST(AHRS, getPitch)
                {

                }
                TEST(AHRS, getRoll)
                {

                }
                TEST(AHRS, getYaw)
                {

                }
                TEST(AHRS, getVelocity)
                {
                    // x, y, z
                }

                TEST(AHRS, getByteCount)
                {

                }
                TEST(AHRS, getUpdateCount)
                {

                }Z
                TEST(AHRS, getLastSensorTimestamp)
                {

                }
                TEST(ARHS, getWorldLinearAccel)
                {
                    // x, y, z
                }
                TEST(AHRS, getBarometricPressure)
                {

                }
                TEST(AHRS, getAltitude)
                {

                }
                TEST(AHRS, getFusedHeading)
                {

                }
                TEST(AHRS, getQuaternion)
                {
                    // w, x, y, z
                }
                TEST(AHRS, getDisplacement)
                {
                    // x, y, z
                }
                TEST(ARHS, getAngle)
                {

                }
                TEST(AHRS, getRate)
                {

                }
                TEST(AHRS, getRawGyro)
                {
                    // x, y, z
                }
                TEST(AHRS, getRawAccel)
                {
                    // x, y, z
                }
                TEST(AHRS, getRawMag)
                {
                    // x, y, z
                }
                TEST(AHRS, getTempC)
                {

                }
                TEST(AHRS, getBoardYawAxis)
                {

                }
                TEST(AHRS, getFirmwareVersion)
                {

                }
                TEST(AHRS, getActualUpdateRate)
                {

                }
                TEST(AHRS, getRequestedUpdateRate)
                {
                    
                }

                /* BOOLS */
                TEST(AHRS, isCalibrating)
                {

                }
                TEST(AHRS, isConnected)
                {

                }
                TEST(AHRS, isMoving)
                {

                }
                TEST(AHRS, isRotating)
                {

                }
                TEST(AHRS, isAltitudeValid)
                {

                }
                TEST(AHRS, isMagneticDisturbance)
                {

                }
                TEST(AHRS, isMagnetometerCalibrated)
                {

                }
                TEST(AHRS, registerCallback)
                {

                }
                TEST(AHRS, deregisterCallback)
                {

                }

                /* OTHER FUNCTIONS */
                TEST(AHRS, reset)
                {

                }
                TEST(resetDisplacement)TEST(AHRS, getCompassHeading)
                {

                }
                {

                }
                TEST(AHRS, updateDisplacement_noargs)
                {

                }
                TEST(AHRS, updateDisplacement_withargs)
                {

                }
            }
        }
    }
}
