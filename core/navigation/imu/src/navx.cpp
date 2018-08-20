#include <imu/navx.hpp>

namespace rip
{
    namespace navigation
    {
        namespace imu
        {
            NavX::NavX(const std::string& name, Nav_x navx) : Imu(name), m_navx(navx)
            {}

            units::Angle NavX::getYaw()
            {
                return m_navx.getYaw();
            }

            units::Angle NavX::getPitch()
            {
                return m_navx.getPitch();
            }

            units::Angle NavX::getRoll()
            {
                return m_navx.getRoll();
            }

            bool NavX::isCalibrated()
            {
                return m_navx.isCalibrating();
            }
            // virtual void zeroYaw();
            // virtual void getAngle();

            void NavX::stop()
            {
                m_navx.stop();
            }

            bool NavX::diagnostic()
            {
                return m_navx.diagnostic();
            }
        }
    }
}