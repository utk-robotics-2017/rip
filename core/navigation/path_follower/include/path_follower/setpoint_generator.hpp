#ifndef SETPOINT_GENERATOR_HPP
#define SETPOINT_GENERATOR_HPP

#include "path_follower/motion_profile.hpp"
#include "path_follower/motion_profile_goal.hpp"
#include "path_follower/motion_profile_constraints.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class Setpoint
            {
            public:
                Setpoint(const MotionState& state, bool final);

                MotionState state() const;
                void setState(const MotionState& state);
                bool final() const;
                void setFinal(bool final);

            private:
                MotionState m_state;
                bool m_final;
            };

            class SetpointGenerator
            {
            public:
                SetpointGenerator();

                void reset();

                /**
                 * Get a new Setpoint (and generate a new MotionProfile if necessary).
                 *
                 * @param constraints
                 *            The constraints to use.
                 * @param goal
                 *            The goal to use.
                 * @param prev_state
                 *            The previous setpoint (or measured state of the system to do a reset).
                 * @param t
                 *            The time to generate a setpoint for.
                 * @return The new Setpoint at time t.
                 */
                Setpoint setpoint(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous, const units::Time& t);

                nonstd::optional<MotionProfile> profile() const;
            private:
                nonstd::optional<MotionProfile> m_profile;
                nonstd::optional<MotionProfileGoal> m_goal;
                nonstd::optional<MotionProfileConstraints> m_constraints;
            };
        }
    }
}

#endif //SETPOINT_GENERATOR_HPP
