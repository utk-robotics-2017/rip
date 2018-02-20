#ifndef POSE_HPP
#define POSE_HPP

#include <units/units.hpp>
#include <geometry/point.hpp>

#include <teb_planner/pose_se2.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            class Pose
            {
            public:
                Pose()
                {}

                Pose(const units::Distance& x, const units::Distance& y, const units::Angle& theta)
                    : m_x(x)
                    , m_y(y)
                    , m_theta(theta)
                {}

                Pose(const PoseSE2& pose)
                    : m_x(pose.x() * units::m)
                    , m_y(pose.y() * units::m)
                    , m_theta(pose.theta() * units::rad)
                {}

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

                geometry::Point orientationUnitVector() const
                {
                    return geometry::Point( units::cos(m_theta), units::sin(m_theta) ).normalize();
                }

                Pose operator+(const Pose& rhs) const
                {
                    return Pose(m_x + rhs.m_x, m_y + rhs.m_y, m_theta + rhs.m_theta);
                }

                Pose& operator+=(const Pose& rhs)
                {
                    m_x += rhs.m_x;
                    m_y += rhs.m_y;
                    m_theta += rhs.m_theta;
                    return *this;
                }

                Pose operator-(const Pose& rhs) const
                {
                    return Pose(m_x - rhs.m_x, m_y - rhs.m_y, m_theta - rhs.m_theta);
                }

                Pose& operator -= (const Pose& rhs)
                {
                    m_x -= rhs.m_x;
                    m_y -= rhs.m_y;
                    m_theta -= rhs.m_theta;
                    return *this;
                }

                operator PoseSE2() const
                {
                    return PoseSE2(m_x.to(units::m), m_y.to(units::m), m_theta.to(units::rad));
                }

            private:
                units::Distance m_x;
                units::Distance m_y;
                units::Angle m_theta;
            };
        }
    }
}
#endif
