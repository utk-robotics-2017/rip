#include <imu/bno055.hpp>

namespace rip
{
    namespace navigation
    {
        namespace imu
        {
            Bno055::Bno055(const std::string& name, Bno bno) : Imu(name), m_bno(bno)
            {}

            units::Angle Bno055::getYaw()
            {
                return m_bno.getYaw();
            }

            units::Angle Bno055::getPitch()
            {
                return m_bno.getPitch();
            }

            units::Angle Bno055::getRoll()
            {
                return m_bno.getRoll();
            }

            units::AngularVelocity Bno055::getRate()
            {
                return m_bno.getRate();
            }

            bool Bno055::isCalibrated()
            {
                return m_bno.isCalibrated();
            }
            // virtual void zeroYaw();
            // virtual void getAngle();

            void Bno055::stop()
            {
                m_bno.stop();
            }

            bool Bno055::diagnostic()
            {
                return m_bno.diagnostic();
            }
        }
    }
}
