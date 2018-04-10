#include "path_follower/twist_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Twist2d::Twist2d(const units::Distance& dx, const units::Distance& dy, const units::Angle& dtheta)
                : m_dx(dx), m_dy(dy), m_dtheta(dtheta)
            {}

            units::Distance Twist2d::dx() const
            {
                return m_dx;
            }

            units::Distance Twist2d::dy() const
            {
                return m_dy;
            }

            units::Angle Twist2d::dtheta() const
            {
                return m_dtheta;
            }

            void Twist2d::setDx(const units::Distance& dx)
            {
                m_dx = dx;
            }

            void Twist2d::setDy(const units::Distance& dy)
            {
                m_dy = dy;
            }

            void Twist2d::setDtheta(const units::Angle& dtheta)
            {
                m_dtheta = dtheta;
            }

            Twist2d Twist2d::scaled(double scale) const
            {
                return Twist2d(m_dx * scale, m_dy * scale, m_dtheta * scale);
            }
        }
    }
}