#ifndef PROFILE_FOLLOWER_HPP
#define PROFILE_FOLLOWER_HPP

#include "path_follower/motion_profile_goal.hpp"
#include "path_follower/motion_profile_constraints.hpp"
#include "path_follower/setpoint_generator.hpp"

#include <optional.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A controller for tracking a profile generated to attain a MotionProfileGoal. The controller uses feedforward on
             * acceleration, velocity, and position; proportional feedback on velocity and position; and integral feedback on
             * position.
             */
            class ProfileFollower
            {
            public:
                ProfileFollower(const double kp, const double ki, const double kv, const double kffv, const double kffa);

                void setGains(const double kp, const double ki, const double kv, const double kffv, const double kffa);

                /**
                 * Completely clear all state related to the current profile (min and max outputs are maintained).
                 */
                void resetProfile();

                /**
                 * Specify a goal and constraints for achieving the goal.
                 */
                void setGoalAndConstraints(const MotionProfileGoal& goal, const MotionProfileConstraints& constraints);

                void setGoal(const MotionProfileGoal& goal);

                MotionProfileGoal goal() const;

                void setConstraints(const MotionProfileConstraints& constraints);

                MotionState setpoint() const;

                /**
                 * Reset just the setpoint. This means that the latest_state provided to update() will be used rather than feeding
                 * forward the previous setpoint the next time update() is called. This almost always forces a MotionProfile update,
                 * and may be warranted if tracking error gets very large.
                 */
                void resetSetpoint();

                void resetIntegral();

                /**
                 * Update the setpoint and apply the control gains to generate a control output.
                 *
                 * @param latest_state
                 *            The latest *actual* state, used only for feedback purposes (unless this is the first iteration or
                 *            reset()/resetSetpoint() was just called, in which case this is the new start state for the profile).
                 * @param t
                 *            The timestamp for which the setpoint is desired.
                 * @return An output that reflects the control output to apply to achieve the new setpoint.
                 */
                virtual double update(const MotionState& latest_state, const units::Time& t);

                void setMinOutput(double min_output);
                void setMaxOutput(double max_output);

                units::Distance getPositionError();
                units::Velocity getVelocityError();

                /**
                 * We are finished the profile when the final setpoint has been generated. Note that this does not check whether we
                 * are anywhere close to the final setpoint, however.
                 *
                 * @return True if the final setpoint has been generated for the current goal.
                 */
                bool finished() const;

                /**
                 * We are on target if our actual state achieves the goal (where the definition of achievement depends on the goal's
                 * completion behavior).
                 *
                 * @return True if we have actually achieved the current goal.
                 */
                bool onTarget() const;

            protected:
                double m_kp;
                double m_ki;
                double m_kv;
                double m_kffv;
                double m_kffa;

                double m_min_output;
                double m_max_output;

                nonstd::optional<MotionState> m_latest_actual_state;
                nonstd::optional<MotionState> m_initial_state;

                units::Distance m_latest_position_error;
                units::Velocity m_latest_velocity_error;
                double m_total_error;

                nonstd::optional<MotionProfileGoal> m_goal;
                nonstd::optional<MotionProfileConstraints> m_constraints;
                SetpointGenerator m_setpoint_generator;
                nonstd::optional<Setpoint> m_latest_setpoint;
            };
        }
    }
}

#endif //PROFILE_FOLLOWER_HPP
