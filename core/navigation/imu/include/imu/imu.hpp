#ifndef IMU_HPP
#define IMU_HPP
#include <units/units.hpp>
#include <framework/subsystem.hpp>
#include <pid/pid_input.hpp>
// Supported IMUs
// #include <navx/navx.hpp>
// #include <appendages/bno055.hpp>

namespace rip
{
    namespace navigation
    {
        namespace imu
        {
            /**
             * Abstract base class for IMUs
             */
            class Imu : public framework::Subsystem, public pid::PidInput
            {
            public:
                Imu(const std::string& name)
                    : Subsystem(name)
                {}

                /**
                 *
                 */
                virtual units::Angle getYaw() = 0;

                /**
                 *
                 */
                virtual units::Angle getPitch() = 0;

                /**
                 *
                 */
                virtual units::Angle getRoll() = 0;

                /**
                 *
                 */
                virtual bool isCalibrated() = 0;

                /**
                 *
                 */
                virtual units::AngularVelocity getRate() = 0;

                // virtual void zeroYaw();
                // virtual void getAngle();
            };
        }
    }
}

#endif // IMU_HPP
