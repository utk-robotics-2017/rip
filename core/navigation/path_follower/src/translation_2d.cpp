#include "path_follower/translation_2d.hpp"
#include "path_follower/rotation_2d.hpp"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Translation2d::Translation2d()
                : m_x(0.0)
                , m_y(0.0)
            {}

            Translation2d::Translation2d(const units::Distance& x, const units::Distance& y)
                        : m_x(x)
                        , m_y(y)
            {}

            Translation2d::Translation2d(const Translation2d& other)
                    :m_x(other.m_x)
                    , m_y(other.m_y)
            {}

            Translation2d::Translation2d(const Translation2d& start, const Translation2d& end)
                : m_x(end.m_x - start.m_x)
                , m_y(end.m_y - start.m_y)
            {}

            units::Distance Translation2d::norm() const
            {
                return units::sqrt(units::pow<2>(m_x) + units::pow<2>(m_y));
            }

            double Translation2d::norm2() const
            {
                return (units::pow<2>(m_x) + units::pow<2>(m_y))();
            }

            units::Distance Translation2d::x() const
            {
                return m_x;
            }

            units::Distance Translation2d::y() const
            {
                return m_y;
            }

            void Translation2d::setX(const units::Distance& x)
            {
                m_x = x;
            }

            void Translation2d::setY(const units::Distance& y)
            {
                m_y = y;
            }

            Translation2d Translation2d::translateBy(const Translation2d& other) const
            {
                return Translation2d(m_x + other.m_x, m_y + other.m_y);
            }

            Translation2d Translation2d::rotateBy(const Rotation2d& rotation) const
            {
                return Translation2d(m_x * rotation.cos() - m_y * rotation.sin(), m_x * rotation.sin() + m_y * rotation.cos());
            }

            Translation2d Translation2d::rotateBy(const units::Angle& angle) const
            {
                return rotateBy(angle);
            }

            Rotation2d Translation2d::direction() const
            {
                return Rotation2d(m_x(), m_y(), true);
            }

            Translation2d Translation2d::inverse() const
            {
                return Translation2d(-m_x, -m_y);
            }

            Translation2d Translation2d::perpendicular() const
            {
                return Translation2d(-m_y, m_x);
            }

            Translation2d Translation2d::interpolate(const Translation2d& other, double x)
            {
                if (x <= 0.0)
                {
                    return Translation2d(*this);
                }
                else if (x >= 1.0)
                {
                    return Translation2d(other);
                }
                return extrapolate(other, x);
            }

            Translation2d Translation2d::extrapolate(const Translation2d& other, double x)
            {
                return Translation2d(x * (other.m_x - m_x) + m_x, x * (other.m_y - m_y) + m_y);
            }
            Translation2d Translation2d::scale(double s) const
            {
                return Translation2d(m_x * s, m_y * s);
            }

            std::string Translation2d::toString() const
            {
                return fmt::format("({} in,{} in)", m_x.to(units::in), m_y.to(units::in));
            }

            std::ostream& operator<<(std::ostream& os, const Translation2d& rhs)
            {
                os << rhs.toString() << std::endl;
                return os;
            }

            double Translation2d::dot(const Translation2d& other)
            {
                return dot(*this, other);
            }

            double Translation2d::dot(const Translation2d& lhs, const Translation2d& rhs)
            {
                return (lhs.m_x * rhs.m_x + lhs.m_y * rhs.m_y)();
            }

            units::Angle Translation2d::getAngle(const Translation2d& lhs, const Translation2d& rhs)
            {
                double cos_angle = dot(lhs, rhs) / (lhs.norm() * rhs.norm())();
                if (std::isnan(cos_angle)) {
                    return Rotation2d().angle();
                }
                return acos(std::min(1.0, std::max(cos_angle, -1.0))) * units::rad;
            }

            double Translation2d::cross(const Translation2d& rhs)
            {
                return cross(*this, rhs);
            }

            double Translation2d::cross(const Translation2d& lhs, const Translation2d& rhs)
            {
                return (lhs.m_x * rhs.m_y - lhs.m_y * rhs.m_x)();
            }

            Translation2d Translation2d::operator-(const Translation2d& rhs) const
            {
                return Translation2d(m_x - rhs.m_x, m_y - rhs.m_y);
            }

            Translation2d& Translation2d::operator-=(const Translation2d& rhs)
            {
                m_x -= rhs.m_x;
                m_y -= rhs.m_y;

                return *this;
            }

            Translation2d Translation2d::operator+(const Translation2d& rhs) const
            {
                return Translation2d(m_x + rhs.m_x, m_y + rhs.m_y);
            }

            Translation2d Translation2d::operator+=(const Translation2d& rhs)
            {
                m_x += rhs.m_x;
                m_y += rhs.m_y;

                return *this;
            }

            Translation2d::operator geometry::Point() const
            {
                return geometry::Point(m_x, m_y);
            }
        }
    }
}
