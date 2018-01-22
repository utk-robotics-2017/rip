#include <teb_planner/velocity_pose.hpp>

namespace rip
{
    namespace navigation
    {
        VelocityPose::VelocityPose(const Velocity& x, const Velocity& y, const AngularVelocity& omega)
            : m_x(x)
            , m_y(y)
            , m_omega(omega)
        {}

        Velocity VelocityPose::x() const
        {
            return m_x;
        }

        void VelocityPose::setX(const Velocity& x)
        {
            m_x = x;
        }

        Velocity VelocityPose::y() const
        {
            return m_y;
        }

        void VelocityPose::setY(const Velocity& y)
        {
            m_y = y;
        }

        AngularVelocity VelocityPose::omega() const
        {
            return m_omega;
        }

        void VelocityPose::setOmega(const AngularVelocity& omega)
        {
            m_omega = omega;
        }

        VelocityPose VelocityPose::operator -() const
        {
            return VelocityPose(-m_x, -m_y, -m_omega);
        }

        VelocityPose VelocityPose::operator +(const VelocityPose& rhs) const
        {
            return VelocityPose(m_x + rhs.m_x, m_y + rhs.m_y, m_omega + rhs.m_omega);
        }

        VelocityPose& VelocityPose::operator +=(const VelocityPose& rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            m_omega += rhs.m_omega;
            return *this;
        }

        VelocityPose VelocityPose::operator -(const VelocityPose& rhs) const
        {
            return VelocityPose(m_x - rhs.m_x, m_y - rhs.m_y, m_omega - rhs.m_omega);
        }

        VelocityPose& VelocityPose::operator -=(const VelocityPose& rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            m_omega -= rhs.m_omega;
            return *this;
        }

        VelocityPose VelocityPose::operator *(double rhs) const
        {
            return VelocityPose(m_x * rhs, m_y * rhs, m_omega * rhs);
        }

        VelocityPose& VelocityPose::operator *=(double rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            m_omega *= rhs;
            return *this;
        }

        VelocityPose VelocityPose::operator /(double rhs) const
        {
            return VelocityPose(m_x / rhs, m_y / rhs, m_omega / rhs);
        }

        VelocityPose& VelocityPose::operator /=(double rhs)
        {
            m_x /= rhs;
            m_y /= rhs;
            m_omega /= rhs;
            return *this;
        }

        bool VelocityPose::operator ==(const VelocityPose& rhs) const
        {
            return m_x == rhs.m_x && m_y == rhs.m_y && m_omega == rhs.m_omega;
        }

        bool VelocityPose::operator !=(const VelocityPose& rhs) const
        {
            return m_x != rhs.m_x || m_y != rhs.m_y || m_omega != rhs.m_omega;
        }

        void from_json(const nlohmann::json& j, VelocityPose& v)
        {
            v.setX(j["x"]);
            v.setY(j["y"]);
            v.setOmega(j["omega"]);
        }

        void to_json(nlohmann::json& j, const VelocityPose& v)
        {
            j = nlohmann::json
            {
                {"x", v.x()},
                {"y", v.y()},
                {"omega", v.omega()}
            };
        }

        Angle atan(const VelocityPose& p)
        {
            return Angle(atan2(p.y()(), p.x()()) * units::rad);
        }

        VelocityPose operator*(double lhs, const VelocityPose& rhs)
        {
            return rhs * lhs;
        }
    }
}
