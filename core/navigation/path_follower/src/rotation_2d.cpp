#include "path_follower/rotation_2d.hpp"
#include "path_follower/epsilon.hpp"
#include "path_follower/translation_2d.hpp"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Rotation2d::Rotation2d()
                : m_cos_angle(1)
                , m_sin_angle(0)
            {}

            Rotation2d::Rotation2d(double x, double y, bool normalize_)
                    : m_cos_angle(x)
                    , m_sin_angle(y)
            {
                if(normalize_)
                {
                    normalize();
                }
            }

            Rotation2d::Rotation2d(const Rotation2d& other)
                : m_cos_angle(other.m_cos_angle)
                , m_sin_angle(other.m_sin_angle)
            {

            }

            Rotation2d::Rotation2d(const Translation2d& direction, bool normalize_)
                    : m_cos_angle(direction.x()())
                    , m_sin_angle(direction.y()())
            {
                if(normalize_)
                {
                    normalize();
                }
            }

            Rotation2d::Rotation2d(const units::Angle& angle)
                : m_cos_angle(units::cos(angle))
                , m_sin_angle(units::sin(angle))
            {}

            void Rotation2d::normalize()
            {
                double magnitude = std::sqrt(std::pow(m_cos_angle, 2) + std::pow(m_sin_angle, 2));
                if(magnitude > k_epsilon)
                {
                    m_sin_angle /= magnitude;
                    m_cos_angle /= magnitude;
                }
                else
                {
                    m_cos_angle = 1;
                    m_sin_angle = 0;
                }
            }

            double Rotation2d::cos() const
            {
                return m_cos_angle;
            }

            double Rotation2d::sin() const
            {
                return m_sin_angle;
            }

            double Rotation2d::tan() const
            {
                if(std::abs(m_cos_angle) < k_epsilon)
                {
                    if(m_sin_angle >= 0.0)
                    {
                        return std::numeric_limits<double>::infinity();
                    }
                    else
                    {
                        return -std::numeric_limits<double>::infinity();
                    }
                }
                return m_sin_angle / m_cos_angle;
            }

            units::Angle Rotation2d::angle() const
            {
                return std::atan2(m_sin_angle, m_cos_angle) * units::rad;
            }

            Rotation2d Rotation2d::rotateBy(const Rotation2d& other) const
            {
                return Rotation2d(m_cos_angle * other.m_cos_angle - m_sin_angle * other.m_sin_angle,
                m_cos_angle * other.m_sin_angle + m_sin_angle * other.m_cos_angle, true);
            }

            Rotation2d Rotation2d::normal() const
            {
                return Rotation2d(-m_sin_angle, m_cos_angle, false);
            }

            Rotation2d Rotation2d::inverse() const
            {
                return Rotation2d(m_cos_angle, -m_sin_angle, false);
            }

            bool Rotation2d::parallel(const Rotation2d& other) const
            {
                return epsilonEquals(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0, k_epsilon);
            }

            Translation2d Rotation2d::toTranslation() const
            {
                return Translation2d(m_cos_angle, m_sin_angle);
            }

            Rotation2d Rotation2d::interpolate(const Rotation2d& other, double x)
            {
                if (x <= 0)
                {
                    return Rotation2d(*this);
                }
                else if (x >= 1)
                {
                    return Rotation2d(other);
                }
                units::Angle angle_diff = inverse().rotateBy(other).angle();
                return this->rotateBy(Rotation2d(angle_diff * x));
            }

            std::string Rotation2d::toString() const
            {
                return fmt::format("{0:0.2f} deg", angle().to(units::deg));
            }

            Rotation2d Rotation2d::operator+(const Rotation2d& rhs) const
            {
                return rotateBy(rhs);
            }

            Rotation2d& Rotation2d::operator+=(const Rotation2d& rhs)
            {
                m_cos_angle = m_cos_angle * rhs.m_cos_angle - m_sin_angle * rhs.m_sin_angle;
                m_sin_angle = m_cos_angle * rhs.m_sin_angle + m_sin_angle * rhs.m_cos_angle;
                normalize();
                return *this;
            }

            Rotation2d Rotation2d::operator-(const Rotation2d& rhs) const
            {
                return rotateBy(rhs.inverse());
            }

            Rotation2d& Rotation2d::operator-=(const Rotation2d& rhs)
            {
                m_cos_angle = m_cos_angle * -rhs.m_cos_angle - m_sin_angle * -rhs.m_sin_angle;
                m_sin_angle = m_cos_angle * -rhs.m_sin_angle + m_sin_angle * -rhs.m_cos_angle;
                normalize();
                return *this;
            }

            std::ostream& operator<<(std::ostream& os, const Rotation2d& rhs)
            {
                os << rhs.toString() << std::endl;
                return os;
            }
        }
    }
}
