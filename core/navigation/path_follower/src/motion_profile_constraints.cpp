#include "path_follower/motion_profile_constraints.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionProfileConstraints::MotionProfileConstraints(const units::Velocity& max_velocity, const units::Acceleration& max_acceleration)
                    : m_max_abs_velocity(max_velocity)
                    , m_max_abs_acceleration(max_acceleration)
            {}

            units::Velocity MotionProfileConstraints::maxVelocity() const
            {
                return m_max_abs_velocity;
            }

            units::Acceleration MotionProfileConstraints::maxAcceleration() const
            {
                return m_max_abs_acceleration;
            }

            bool MotionProfileConstraints::operator==(const MotionProfileConstraints& rhs)
            {
                return m_max_abs_acceleration == rhs.m_max_abs_acceleration && m_max_abs_velocity == rhs.m_max_abs_velocity;
            }

            bool MotionProfileConstraints::operator!=(const MotionProfileConstraints& rhs)
            {
                return m_max_abs_acceleration != rhs.m_max_abs_acceleration || m_max_abs_velocity != rhs.m_max_abs_velocity;
            }
        }
    }
}