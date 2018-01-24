#include <teb_planner/pose.hpp>

namespace rip
{
    namespace navigation
    {

        Pose::Pose(const units::Distance& x, const units::Distance& y, const units::Angle& theta)
            : m_x(x)
            , m_y(y)
            , m_theta(theta)
        {

        }

        Pose::Pose(const geometry::Point& xy, const units::Angle& theta)
            : m_x(xy.x())
            , m_y(xy.y())
            , m_theta(theta)
        {}

        geometry::Point Pose::position() const
        {
            return geometry::Point(m_x, m_y);
        }

        void Pose::setPosition(const geometry::Point& p)
        {
            m_x = p.x();
            m_y = p.y();
        }

        units::Distance Pose::x() const
        {
            return m_x;
        }

        void Pose::setX(const units::Distance& x)
        {
            m_x = x;
        }

        units::Distance Pose::y() const
        {
            return m_y;
        }

        void Pose::setY(const units::Distance& y)
        {
            m_y = y;
        }

        units::Angle Pose::orientation() const
        {
            return theta();
        }

        geometry::Point Pose::orientationUnitVector() const
        {
            return geometry::Point(units::cos(m_theta), units::sin(m_theta));
        }

        units::Angle Pose::theta() const
        {
            int revs = m_theta.to(units::rev);
            return m_theta - (revs + 0.5) * units::rev;
        }

        void Pose::setTheta(const units::Angle& theta)
        {
            m_theta = theta;
        }

        Pose Pose::operator +(const Pose& rhs) const
        {
            return Pose(m_x + rhs.m_x, m_y + rhs.m_y, m_theta + rhs.m_theta);
        }

        Pose&Pose::operator +=(const Pose& rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            m_theta += rhs.m_theta;
            return *this;
        }

        Pose Pose::operator -(const Pose& rhs) const
        {
            return Pose(m_x - rhs.m_x, m_y - rhs.m_y, m_theta - rhs.m_theta);
        }

        Pose&Pose::operator -=(const Pose& rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            m_theta -= rhs.m_theta;
            return *this;
        }

        Pose Pose::operator *(double rhs) const
        {
            return Pose(m_x * rhs, m_y * rhs, m_theta * rhs);
        }

        Pose&Pose::operator *=(double rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            m_theta *= rhs;
            return *this;
        }

        Pose Pose::operator /(double rhs) const
        {
            return Pose(m_x / rhs, m_y / rhs, m_theta / rhs);
        }

        Pose Pose::operator /=(double rhs)
        {
            m_x /= rhs;
            m_y /= rhs;
            m_theta /= rhs;
            return *this;
        }

        bool Pose::operator ==(const Pose& rhs) const
        {
            return m_x == rhs.m_x && m_y == rhs.m_y && m_theta == rhs.m_theta;
        }

        bool Pose::operator !=(const Pose& rhs) const
        {
            return m_x != rhs.m_x || m_y != rhs.m_y || m_theta != rhs.m_theta;
        }

    }
}
