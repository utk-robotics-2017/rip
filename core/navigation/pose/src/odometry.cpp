#include "pose/odometry.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Odometry::Odometry(const units::Distance& wheelbase)
                : m_wheelbase(wheelbase)
                , m_x(0)
                , m_y(0)
                , m_heading(0)

            {
            }

            void Odometry::update(const units::Time& time, const units::Distance& left, const units::Distance& right)
            {
                const units::Time dt = time - m_time;
                update(time, left / dt, right /dt);
            }

            void Odometry::update(const units::Time& time, const units::Velocity& left, const units::Velocity& right)
            {
                const units::Time dt = time - m_time;

                // Robot Frame
                const units::Velocity v_rx = (left + right) / 2.0;
                const units::Velocity v_ry = 0; // (for a holonomic robot, this needs to be changed)
                const units::AngularVelocity omega_r = ((right - left).to(units::m / units::s) / m_wheelbase.to(units::m)) * units::rad / units::s;

                // Odometry Frame
                const units::Velocity v_wx = v_rx * cos(m_theta) - v_ry * sin(m_theta);
                const units::Velocity v_wy = v_rx * sin(m_theta) + v_ry * cos(m_theta);
                const units::AngularVelocity theta_dot = omega_r;

                // Pose
                m_x += v_wx * dt;
                m_y += v_wy * dt;
                m_theta += theta_dot * dt;

                m_dx = v_wx;
                m_dy = v_wy;
                m_dtheta = theta_dot;

                m_linear = v_rx;
                m_angular = omega_r;
            }

            units::Time Odometry::time() const
            {
                return m_time;
            }

            units::Distance Odometry::x() const
            {
                return m_x;
            }

            units::Distance Odometry::y() const
            {
                return m_y;
            }

            units::Angle Odometry::theta() const
            {
                return m_theta;
            }

            units::Velocity Odometry::dx() const
            {
                return m_dx;
            }

            units::Velocity Odometry::dy() const
            {
                return m_dy;
            }

            units::AngularVelocity Odometry::dtheta() const
            {
                return m_dtheta;
            }

            units::Velocity Odometry::linear() const
            {
                return m_linear;
            }

            units::AngularVelocity Odometry::angular() const
            {
                return m_angular;
            }
        }
    }
}