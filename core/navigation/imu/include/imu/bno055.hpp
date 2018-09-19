
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
                ~Bno055();

                /**
                 *
                 */
                virtual units::Angle getYaw() override;
                /**
                 *
                 */
                virtual units::Angle getPitch() override;
                /**
                 *
                 */
                virtual units::Angle getRoll() override;

                /**
                 *
                 */
                virtual units::AngularVelocity getRate() override;

                /**
                 *
                 */
                virtual double get() override;

                /**
                 *
                 */
                virtual bool isCalibrated() override;
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
