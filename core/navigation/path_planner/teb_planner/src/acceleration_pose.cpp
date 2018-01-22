#include <teb_planner/acceleration_pose.hpp>

namespace rip
{
    namespace navigation
    {
        AccelerationPose::AccelerationPose(const units::Acceleration& x, const units::Acceleration& y, const units::AngularAcceleration& alpha)
            : m_x(x)
            , m_y(y)
            , m_alpha(alpha)
        {
        }

        units::Acceleration AccelerationPose::x() const
        {
            return m_x;
        }

        void AccelerationPose::setX(const units::Acceleration& x)
        {
            m_x = x;
        }

        units::Acceleration AccelerationPose::y() const
        {
            return m_y;
        }

        void AccelerationPose::setY(const units::Acceleration& y)
        {
            m_y = y;
        }

        units::AngularAcceleration AccelerationPose::alpha() const
        {
            return m_alpha;
        }

        void AccelerationPose::setAlpha(const units::AngularAcceleration& alpha)
        {
            m_alpha = alpha;
        }

        AccelerationPose AccelerationPose::operator +(const AccelerationPose& rhs) const
        {
            return AccelerationPose(m_x + rhs.m_x, m_y + rhs.m_y, m_alpha + rhs.m_alpha);
        }

        AccelerationPose& AccelerationPose::operator +=(const AccelerationPose& rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            m_alpha += rhs.m_alpha;
            return *this;
        }

        AccelerationPose AccelerationPose::operator -(const AccelerationPose& rhs) const
        {
            return AccelerationPose(m_x - rhs.m_x, m_y - rhs.m_y, m_alpha - rhs.m_alpha);
        }

        AccelerationPose& AccelerationPose::operator -=(const AccelerationPose& rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            m_alpha -= rhs.m_alpha;
            return *this;
        }

        AccelerationPose AccelerationPose::operator *(double rhs) const
        {
            return AccelerationPose(m_x * rhs, m_y * rhs, m_alpha * rhs);
        }

        AccelerationPose& AccelerationPose::operator *=(double rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            m_alpha *= rhs;
            return *this;
        }

        bool AccelerationPose::operator ==(const AccelerationPose& rhs) const
        {
            return m_x == rhs.m_x && m_y == rhs.m_y && m_alpha == rhs.m_alpha;
        }

        bool AccelerationPose::operator !=(const AccelerationPose& rhs) const
        {
            return m_x != rhs.m_x || m_y != rhs.m_y || m_alpha != rhs.m_alpha;
        }
    }
}
