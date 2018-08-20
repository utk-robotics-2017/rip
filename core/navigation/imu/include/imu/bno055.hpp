
// Supported IMUs
// #include <navx/navx.hpp>
#include <appendages/bno055.hpp>
#include "imu.hpp"

namespace rip
{
    using Bno = appendages::Bno055;
    namespace navigation
    {
        namespace imu
        {
            /**
             * Abstract base class for the drive train
             */

            class Bno055 : public Imu
            {
            public:
                Bno055(const std::string& name, Bno bno) ;
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
                Bno m_bno;
            };
        }
    }
}

// #endif // IMU_HPP
