#ifndef MOTION_PROFILE_GENERATOR_HPP
#define MOTION_PROFILE_GENERATOR_HPP

#include "path_follower/motion_profile.hpp"
#include "path_follower/motion_profile_constraints.hpp"
#include "path_follower/motion_profile_goal.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class MotionProfileGenerator
            {
            public:
                static MotionProfile generateProfile(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous);
            protected:
                static MotionProfile generateFlippedProfile(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous);
            private:
                MotionProfileGenerator() = default;
            };
        }
    }
}

#endif //MOTION_PROFILE_GENERATOR_HPP
