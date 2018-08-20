#ifndef NAVX_HPP
#define NAVX_HPP

// Supported IMUs
#include <navx/navx.hpp>
// #include <appendages/bno055.hpp>
#include "imu.hpp"

namespace rip
{
    namespace navigation
    {
        using Nav_x = navx::NavX;

        namespace imu
        {
            /** 
             * NavX abstract implementation
             */
            class NavX : public Imu
            {
            public:
                NavX(const std::string& name, Nav_x navx);
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

                virtual void stop() override;

                virtual bool diagnostic() override;
            private:
                Nav_x m_navx;
            };
        }
    }
}

#endif // IMU_HPP
