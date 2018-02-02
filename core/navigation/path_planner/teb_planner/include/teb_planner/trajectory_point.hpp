#ifndef TRAJECTORY_POINT_HPP
#define TRAJECTORY_POINT_HPP

#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            class TrajectoryPoint
            {
            public:
                TrajectoryPoint()
                {}

                TrajectoryPoint(const fakeros::TrajectoryPointMsg& msg)
                {
                    m_t = msg.time_from_start.sec * units::s + msg.time_from_start.nsec * units::nano * units::s;
                    m_x = msg.pose.position.x * units::m;
                    m_y = msg.pose.position.y * units::m;
                    m_theta = msg.pose.orientation.z * units::rad;
                    m_dx = msg.velocity.linear.x * units::m / units::s;
                    m_dy = msg.velocity.linear.y  * units::m / units::s;
                    m_omega = msg.velocity.angular.z * units::rad / units::s;
                    m_ddx = msg.acceleration.linear.x * units::m / units::s / units::s;
                    m_ddy = msg.acceleration.linear.y * units::m / units::s / units::s;
                    m_alpha = msg.acceleration.angular.z * units::rad / units::s / units::s;
                }

                units::Time t() const
                {
                    return m_t;
                }

                units::Distance x() const
                {
                    return m_x;
                }

                void setX(const units::Distance& x)
                {
                    m_x = x;
                }

                units::Distance y() const
                {
                    return m_y;
                }

                void setY(const units::Distance& y)
                {
                    m_y = y;
                }

                geometry::Point position() const
                {
                    return geometry::Point(m_x, m_y);
                }

                void setPosition(const units::Distance& x, const units::Distance& y)
                {
                    m_x = x;
                    m_y = y;
                }

                void setPosition(const geometry::Point& p)
                {
                    m_x = p.x();
                    m_y = p.y();
                }

                units::Angle theta() const
                {
                    return m_theta;
                }

                void setTheta(const units::Angle& theta)
                {
                    m_theta = theta;
                }

                units::Angle orientation() const
                {
                    return m_theta;
                }

                void setOrientation(const units::Angle& theta)
                {
                    m_theta = theta;
                }

                units::Velocity dx() const
                {
                    return m_dx;
                }

                void setDx(const units::Velocity& dx)
                {
                    m_dx = dx;
                }

                units::Velocity dy() const
                {
                    return m_dy;
                }

                void setDy(const units::Velocity& dy)
                {
                    m_dy = dy;
                }

                units::AngularVelocity omega() const
                {
                    return m_omega;
                }

                void setOmega(const units::AngularVelocity& omega)
                {
                    m_omega = omega;
                }

                units::Acceleration ddx() const
                {
                    return m_ddx;
                }

                void setDdx(const units::Acceleration& ddx)
                {
                    m_ddx = ddx;
                }

                units::Acceleration ddy() const
                {
                    return m_ddy;
                }

                void setDdy(const units::Acceleration& ddy)
                {
                    m_ddy = ddy;
                }

                units::AngularAcceleration alpha() const
                {
                    return m_alpha;
                }

                void setAlpha(const units::AngularAcceleration& alpha)
                {
                    m_alpha = alpha;
                }

            private:
                units::Time m_t;
                units::Distance m_x, m_y;
                units::Angle m_theta;
                units::Velocity m_dx, m_dy;
                units::AngularVelocity m_omega;
                units::Acceleration m_ddx, m_ddy;
                units::AngularAcceleration m_alpha;
            };
        }
    }
}

#endif // TRAJECTORY_POINT_HPP
