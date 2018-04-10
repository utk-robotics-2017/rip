#include "path_follower/profile_follower.hpp"
#include <limits>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            ProfileFollower::ProfileFollower(const double kp, const double ki, const double kv, const double kffv, const double kffa)
                    : m_min_output(-std::numeric_limits<double>::infinity()), m_max_output(std::numeric_limits<double>::infinity())
            {
                resetProfile();
                setGains(kp, ki, kv, kffv, kffa);
            }

            void ProfileFollower::setGains(const double kp, const double ki, const double kv, const double kffv, const double kffa)
            {
                m_kp = kp;
                m_ki = ki;
                m_kv = kv;
                m_kffv = kffv;
                m_kffa = kffa;
            }

            void ProfileFollower::resetProfile()
            {
                m_total_error = 0.0;
                m_initial_state = MotionState::k_invalid_state;
                m_latest_actual_state = MotionState::k_invalid_state;
                m_latest_position_error = std::nan("");
                m_latest_velocity_error = std::nan("");
                m_setpoint_generator.reset();
                resetSetpoint();
            }

            void ProfileFollower::setGoalAndConstraints(const MotionProfileGoal& goal, const MotionProfileConstraints& constraints)
            {
                if(!m_goal && m_goal.value() != goal && !m_latest_setpoint)
                {
                    // Clear the final state bit since the goal has changed
                    m_latest_setpoint.value().setFinal(false);
                }
                m_goal = goal;
                m_constraints = constraints;
            }

            void ProfileFollower::setGoal(const MotionProfileGoal& goal)
            {
                setGoalAndConstraints(goal, m_constraints.value());
            }

            MotionProfileGoal ProfileFollower::goal() const
            {
                return m_goal.value();
            }

            void ProfileFollower::setConstraints(const MotionProfileConstraints& constraints)
            {
                setGoalAndConstraints(m_goal.value(), constraints);
            }

            MotionState ProfileFollower::setpoint() const
            {
                return !m_latest_setpoint ? MotionState::k_invalid_state : m_latest_setpoint.value().state();
            }

            void ProfileFollower::resetSetpoint()
            {
                m_latest_setpoint = nonstd::nullopt;
            }

            void ProfileFollower::resetIntegral()
            {
                m_total_error = 0.0;
            }

            double ProfileFollower::update(const MotionState& latest_state, const units::Time& t)
            {
                m_latest_actual_state = latest_state;
                MotionState previous = latest_state;
                if(m_latest_setpoint)
                {
                    previous = m_latest_setpoint.value().state();
                }
                else
                {
                    m_initial_state = previous;
                }

                const units::Time dt = units::max(units::Time(0.0), t - previous.t());
                m_latest_setpoint = m_setpoint_generator.setpoint(m_constraints.value(), m_goal.value(), previous, t);

                // Update error
                m_latest_position_error = m_latest_setpoint.value().state().position() - latest_state.position();
                m_latest_velocity_error = m_latest_setpoint.value().state().velocity() - latest_state.velocity();

                // Calculate the feedforward and proportional terms.
                double output = m_kp * m_latest_position_error() +
                        m_kv * m_latest_velocity_error() +
                        m_kffv * m_latest_setpoint.value().state().velocity()()
                    + (units::isnan(m_latest_setpoint.value().state().acceleration()) ? 0.0 : m_kffa * m_latest_setpoint.value().state().acceleration()());

                if(output >= m_min_output && output <= m_max_output)
                {
                    // Update integral
                    m_total_error += (m_latest_position_error * dt)();
                    output += m_ki * m_total_error;
                }
                else
                {
                    // Reset integral windup
                    m_total_error = 0.0;
                }

                output = std::max(m_min_output, std::min(m_max_output, output));

                return output;
            }

            void ProfileFollower::setMinOutput(double min_output)
            {
                m_min_output = min_output;
            }

            void ProfileFollower::setMaxOutput(double max_output)
            {
                m_max_output = max_output;
            }

            units::Distance ProfileFollower::getPositionError()
            {
                return m_latest_position_error;
            }

            units::Velocity ProfileFollower::getVelocityError()
            {
                return m_latest_velocity_error;
            }

            bool ProfileFollower::finished() const
            {
                return m_goal && m_latest_setpoint && m_latest_setpoint.value().final();
            }

            bool ProfileFollower::onTarget() const
            {
                if(!m_goal || !m_latest_setpoint)
                {
                    return false;
                }

                // For the options that don't achieve the goal velocity exactly, also count any instance where we have passed
                // the finish line.
                const units::Distance goal_to_start = m_goal.value().position() - m_initial_state.value().position();
                const units::Distance goal_to_actual = m_goal.value().position() - m_latest_actual_state.value().position();
                const bool passed_goal_state = units::signum(goal_to_start) * units::signum(goal_to_actual) < 0.0;
                return m_goal.value().atGoalState(m_latest_actual_state.value()) || (m_goal.value().completionBehavior() != MotionProfileGoal::CompletionBehavior::kOvershoot && passed_goal_state);
            }
        }
    }
}