#include "path_follower/rigid_transform_2d.hpp"
#include "path_follower/epsilon.hpp"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            RigidTransform2d::RigidTransform2d(const Translation2d& translation, const Rotation2d& rotation)
                    : m_translation(translation), m_rotation(rotation)
            {
            }

            RigidTransform2d::RigidTransform2d(const RigidTransform2d& other)
                    : m_translation(other.m_translation), m_rotation(other.m_rotation)
            {
            }

            RigidTransform2d::RigidTransform2d(const pose::Pose& pose)
                : m_translation(pose.x, pose.y), m_rotation(pose.yaw)
            {
            }

            RigidTransform2d RigidTransform2d::fromTranslation(const Translation2d& translation)
            {
                return RigidTransform2d(translation, Rotation2d());
            }

            RigidTransform2d RigidTransform2d::fromRotation(const Rotation2d& rotation)
            {
                return RigidTransform2d(Translation2d(), rotation);
            }

            RigidTransform2d RigidTransform2d::exp(const Twist2d& delta)
            {
                double sin_theta = units::sin(delta.dtheta());
                double cos_theta = units::cos(delta.dtheta());
                double s, c;
                if(units::abs(delta.dtheta()) < 1e-9)
                {
                    s = 1.0 - 1.0 / 6.0 * (delta.dtheta().to(units::rad) * delta.dtheta().to(units::rad));
                    c = 0.5 * delta.dtheta().to(units::rad);
                }
                else
                {
                    s = sin_theta / delta.dtheta().to(units::rad);
                    c = (1.0 - cos_theta) / delta.dtheta().to(units::rad);
                }
                return RigidTransform2d(Translation2d(delta.dx() * s - delta.dy() * c, delta.dx() * c + delta.dy() * s), Rotation2d(cos_theta, sin_theta, false));
            }

            Twist2d RigidTransform2d::log(const RigidTransform2d& transform)
            {
                const units::Angle dtheta = transform.rotation().angle();
                const units::Angle half_dtheta = 0.5 * dtheta;
                const double cos_minus_one = units::cos(dtheta) - 1.0;
                double halftheta_by_tan_of_halfdtheta;
                if(std::abs(cos_minus_one) < 1e-9)
                {
                    halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta.to(units::rad) * dtheta.to(units::rad);
                }
                else
                {
                    halftheta_by_tan_of_halfdtheta = -(half_dtheta.to(units::rad) * units::sin(dtheta)) / cos_minus_one;
                }
                const Translation2d translation_part = transform.translation().rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta.to(units::rad), false));
                return Twist2d(translation_part.x(), translation_part.y(), dtheta);
            }

            Translation2d RigidTransform2d::translation() const
            {
                return m_translation;
            }

            Rotation2d RigidTransform2d::rotation() const
            {
                return m_rotation;
            }

            void RigidTransform2d::setTranslation(const Translation2d& translation)
            {
                m_translation = translation;
            }

            void RigidTransform2d::setRotation(const Rotation2d& rotation)
            {
                m_rotation = rotation;
            }

            RigidTransform2d RigidTransform2d::transformBy(const RigidTransform2d& other) const
            {
                return RigidTransform2d(m_translation.translateBy(other.m_translation.rotateBy(m_rotation)), m_rotation.rotateBy(other.m_rotation));
            }

            RigidTransform2d RigidTransform2d::inverse() const
            {
                const Rotation2d rotation_inverse = m_rotation.inverse();
                return RigidTransform2d(m_translation.inverse().rotateBy(rotation_inverse), rotation_inverse);
            }

            RigidTransform2d RigidTransform2d::normal() const
            {
                return RigidTransform2d(m_translation, m_rotation.normal());
            }

            Translation2d RigidTransform2d::intersection(const RigidTransform2d& other) const
            {
                const Rotation2d other_rotation = other.rotation();
                if(m_rotation.parallel(other_rotation))
                {
                    // Lines are parallel
                    return Translation2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
                }
                if(std::abs(m_rotation.cos()) < std::abs(other_rotation.cos()))
                {
                    return intersectionInternal(*this, other);
                }
                else
                {
                    return intersectionInternal(other, *this);
                }
            }

            bool RigidTransform2d::colinear(const RigidTransform2d& other) const
            {
                const Twist2d twist = log(inverse().transformBy(other));
                return (epsilonEquals<units::Distance>(twist.dy(), 0.0, k_epsilon) && epsilonEquals<units::Angle>(twist.dtheta(), 0.0, k_epsilon));
            }

            RigidTransform2d RigidTransform2d::interpolate(const RigidTransform2d& other, double x)
            {
                if(x <= 0.0)
                {
                    return RigidTransform2d(*this);
                }
                else if(x >= 1.0)
                {
                    return RigidTransform2d(other);
                }

                const Twist2d twist = RigidTransform2d::log(inverse().transformBy(other));
                return transformBy(RigidTransform2d::exp(twist.scaled(x)));
            }

            std::string RigidTransform2d::toString() const
            {
                return fmt::format("T: {}, R: {}", m_translation.toString(), m_rotation.toString());
            }

            RigidTransform2d RigidTransform2d::operator+(const RigidTransform2d& rhs) const
            {
                return RigidTransform2d(m_translation + rhs.m_translation, m_rotation + rhs.m_rotation);
            }

            RigidTransform2d&RigidTransform2d::operator+=(const RigidTransform2d& rhs)
            {
                m_translation += rhs.m_translation;
                m_rotation += rhs.m_rotation;
                return *this;
            }

            std::ostream& operator<<(std::ostream& os, const RigidTransform2d& transform)
            {
                os << transform.toString() << std::endl;
                return os;
            }

            Translation2d RigidTransform2d::intersectionInternal(const RigidTransform2d& lhs, const RigidTransform2d& rhs)
            {
                const Rotation2d lhs_r = lhs.rotation();
                const Rotation2d rhs_r = rhs.rotation();
                const Translation2d lhs_t = lhs.translation();
                const Translation2d rhs_t = rhs.translation();

                const double tan_rhs = rhs_r.tan();
                const double t = ((lhs_t.x() - rhs_t.x()) * tan_rhs + rhs_t.y() - lhs_t.y())() / (lhs_r.sin() - lhs_r.cos() * tan_rhs);
                return lhs_t.translateBy(lhs_r.toTranslation().scale(t));
            }
        }
    }
}
