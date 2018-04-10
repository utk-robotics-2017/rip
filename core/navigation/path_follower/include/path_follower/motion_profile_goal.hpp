#ifndef MOTION_PROFILE_GOAL_HPP
#define MOTION_PROFILE_GOAL_HPP

#include <ostream>

#include <units/units.hpp>

#include "path_follower/motion_state.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A MotionProfileGoal defines a desired position and maximum velocity (at this position), along with the
             * behavior that should be used to determine if we are at the goal and what to do if it is infeasible to reach
             * the goal within the desired velocity bounds.
             */
            class MotionProfileGoal
            {
            public:
                /**
                 *  A goal consists of a desired position and specified maximum velocity magnitude. This enum allows a
                 *  user to specify a preference on the behavior in the case that the robot would reach the goal at a
                 *  velocity greater than the maximum
                 *
                 *  Example use-cases:
                 *
                 *  Overshoot - Generally used with a goal max_abs_vel of 0.0 to stop at the desired position without
                 *      violating the constraints.
                 *
                 *  Violate Max Accel - If we absolutely do not want to pass the goal and are unwilling to violate the
                 *  max_abs_vel (e.g. there is an obstacle in front of us - slam brakes harder than we'd like in order
                 *  to avoid hitting it).
                 *
                 *  Violate Max Vel - If the max velocity is just a general guidelines and not a hard performance limit,
                 *  it's better to slightly exceed it to avoid skidding wheels.
                 */
                enum class CompletionBehavior
                {
                    /**
                     * Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back.
                     * Only valid if the goal velocity is 0.0
                     */
                    kOvershoot,
                    /**
                     * If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max
                     * accel constraint
                     */
                    kViolateMaxAccel,
                    /**
                     * If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the goal
                     * velocity
                     */
                    kViolateMaxVel
                };

                MotionProfileGoal();
                MotionProfileGoal(const units::Distance& position);
                MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel);
                MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel, CompletionBehavior completion_behavior);
                MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel, CompletionBehavior completion_behavior, const units::Distance& position_tolerance, const units::Velocity& velocity_tolerance);
                MotionProfileGoal(const MotionProfileGoal& other);

                MotionProfileGoal flipped() const;

                units::Distance position() const;
                units::Velocity maxVelocity() const;
                units::Distance positionTolerance() const;
                units::Velocity velocityTolerance() const;

                CompletionBehavior completionBehavior() const;

                bool atGoalState(const MotionState& state) const;
                bool atGoalPosition(const units::Distance& position) const;

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream& os, const MotionProfileGoal& goal);

                bool operator==(const MotionProfileGoal& rhs);
                bool operator!=(const MotionProfileGoal& rhs);

            private:
                void sanityCheck();

                std::string toString(CompletionBehavior completion_behavior) const;

                units::Distance m_position;
                units::Velocity m_max_abs_velocity;
                CompletionBehavior m_completion_behavior;
                units::Distance m_position_tolerance;
                units::Velocity m_velocity_tolerance;
            };

            std::ostream& operator<<(std::ostream& os, const MotionProfileGoal& goal);
        }
    }
}

#endif //MOTION_PROFILE_GOAL_HPP