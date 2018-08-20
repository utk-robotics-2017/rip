#ifndef IMU_HPP
#define IMU_HPP
#include <units/units.hpp>
#include <framework/subsystem.hpp>
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
            class Imu : public framework::Subsystem
            {
            public:
                Imu(const std::string& name)
                    : Subsystem(name)
                {}
                /**
                 * 
                 */
                virtual units::Angle getYaw();
                /**
                 * 
                 */
                virtual units::Angle getPitch();
                /**
                 * 
                 */
                virtual units::Angle getRoll();
                /**
                 * 
                 */
                virtual bool isCalibrated();
                // virtual void zeroYaw();
                // virtual void getAngle();
            };
        }
    }
}

#endif // IMU_HPP
