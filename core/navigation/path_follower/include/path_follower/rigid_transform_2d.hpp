#ifndef RIGID_TRANSFORM_2D_HPP
#define RIGID_TRANSFORM_2D_HPP

#include <pose/pose.hpp>

#include "path_follower/interpolable.hpp"
#include "path_follower/translation_2d.hpp"
#include "path_follower/rotation_2d.hpp"
#include "path_follower/twist_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class RigidTransform2d : public Interpolable<RigidTransform2d>
            {
            public:
                RigidTransform2d() = default;
                RigidTransform2d(const Translation2d& translation, const Rotation2d& rotation);
                RigidTransform2d(const RigidTransform2d& other);
                RigidTransform2d(const pose::Pose& pose);

                static RigidTransform2d fromTranslation(const Translation2d& translation);
                static RigidTransform2d fromRotation(const Rotation2d& rotation);

                static RigidTransform2d exp(const Twist2d& delta);
                static Twist2d log(const RigidTransform2d& transform);

                Translation2d translation() const;
                Rotation2d rotation() const;

                void setTranslation(const Translation2d& translation);
                void setRotation(const Rotation2d& rotation);

                RigidTransform2d transformBy(const RigidTransform2d& other) const;
                RigidTransform2d inverse() const;
                RigidTransform2d normal() const;

                Translation2d intersection(const RigidTransform2d& other) const;
                bool colinear(const RigidTransform2d& other) const;
                RigidTransform2d interpolate(const RigidTransform2d& other, double x) override;

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream& os, const RigidTransform2d& transform);

                RigidTransform2d operator+(const RigidTransform2d& rhs) const
                {
                    return RigidTransform2d(m_translation + rhs.m_translation, m_rotation + rhs.m_rotation);
                }

                RigidTransform2d& operator+=(const RigidTransform2d& rhs)
                {
                    m_translation += rhs.m_translation;
                    m_rotation += rhs.m_rotation;
                }

                RigidTransform2d operator-(const RigidTransform2d& rhs) const;
                RigidTransform2d& operator-=(const RigidTransform2d& rhs);

            private:
                static Translation2d intersectionInternal(const RigidTransform2d& lhs, const RigidTransform2d& rhs);

                Translation2d m_translation;
                Rotation2d m_rotation;
            };

            std::ostream& operator<<(std::ostream& os, const RigidTransform2d& transform);
        }
    }
}

#endif //RIGID_TRANSFORM_2D_HPP
