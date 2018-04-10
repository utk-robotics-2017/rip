#ifndef MOTION_PROFILE_CONSTRAINTS_HPP
#define MOTION_PROFILE_CONSTRAINTS_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class MotionProfileConstraints
            {
            public:
                MotionProfileConstraints(const units::Velocity& max_velocity, const units::Acceleration& max_acceleration);

                units::Velocity maxVelocity() const;
                units::Acceleration maxAcceleration() const;

                bool operator==(const MotionProfileConstraints& rhs);
                bool operator!=(const MotionProfileConstraints& rhs);

            private:
                units::Velocity m_max_abs_velocity;
                units::Acceleration m_max_abs_acceleration;
            };
        }
    }
}

#endif //MOTION_PROFILE_CONSTRAINTS_HPP
