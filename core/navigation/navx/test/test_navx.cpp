#include <memory>
#include <gtest/gtest.h>

#include <navx/exceptions.h>
#include "mock_navx.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace tests
            {
                TEST(NavX, SerialConstructor1)
                {
                    // do me
                }

                TEST(NavX, SerialConstructor2)
                {
                    //
                }

                TEST(NavX, zeroYaw)
                {

                }

                TEST(NavX, LinearAccel)
                {

                }

                TEST(NavX, Quaternion)
                {

                }

                /* GETS */
                TEST(NavX, getCompassHeading)
                {

                }

                TEST(NavX, getPitch)
                {

                }

                TEST(NavX, getRoll)
                {

                }

                TEST(NavX, getYaw)
                {

                }

                TEST(NavX, getVelocity)
                {
                    // x, y, z
                }

                TEST(NavX, getByteCount)
                {

                }

                TEST(NavX, getUpdateCount)
                {

                }

                TEST(NavX, getLastSensorTimestamp)
                {

                }

                TEST(NavX, getWorldLinearAccel)
                {
                    // x, y, z
                }

                TEST(NavX, getBarometricPressure)
                {

                }

                TEST(NavX, getAltitude)
                {

                }

                TEST(NavX, getFusedHeading)
                {

                }

                TEST(NavX, getQuaternion)
                {
                    // w, x, y, z
                }

                TEST(NavX, getDisplacement)
                {
                    // x, y, z
                }

                TEST(ARHS, getAngle)
                {

                }

                TEST(NavX, getRate)
                {

                }

                TEST(NavX, getRawGyro)
                {
                    // x, y, z
                }

                TEST(NavX, getRawAccel)
                {
                    // x, y, z
                }

                TEST(NavX, getRawMag)
                {
                    // x, y, z
                }

                TEST(NavX, getTempC)
                {

                }

                TEST(NavX, getBoardYawAxis)
                {

                }

                TEST(NavX, getFirmwareVersion)
                {

                }

                TEST(NavX, getActualUpdateRate)
                {

                }

                TEST(NavX, getRequestedUpdateRate)
                {

                }

                /* BOOLS */
                TEST(NavX, isCalibrating)
                {

                }

                TEST(NavX, isConnected)
                {

                }

                TEST(NavX, isMoving)
                {

                }

                TEST(NavX, isRotating)
                {

                }

                TEST(NavX, isAltitudeValid)
                {

                }

                TEST(NavX, isMagneticDisturbance)
                {

                }

                TEST(NavX, isMagnetometerCalibrated)
                {

                }

                TEST(NavX, registerCallback)
                {

                }

                TEST(NavX, deregisterCallback)
                {

                }

                /* OTHER FUNCTIONS */
                TEST(NavX, reset)
                {

                }

                TEST(NavX, resetDisplacement)
                {

                }

                TEST(NavX, updateDisplacement_noargs)
                {

                }

                TEST(NavX, updateDisplacement_withargs)
                {

                }
            }
        }
    }
}
