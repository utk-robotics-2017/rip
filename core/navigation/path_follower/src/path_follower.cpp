#include "path_follower/path_follower.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            PathFollower::PathFollower(const Path& path, bool reversed, const Parameters& parameters)
                : m_steering_controller(path, reversed, parameters.lookahead)
                , m_velocity_controller(parameters.profile_kp, parameters.profile_ki, parameters.profile_kv, parameters.profile_kffv, parameters.profile_kffa)
                , m_max_profile_velocity(parameters.profile_max_velocity)
                , m_max_profile_acceleration(parameters.profile_max_acceleration)
                , m_goal_position_tolerance(parameters.goal_position_tolerance)
                , m_goal_velocity_tolerance(parameters.goal_velocity_tolerance)
                , m_inertia_gain(parameters.inertia_gain)
                , m_stop_steering_distance(parameters.stop_steering_distance)
            {
                m_velocity_controller.setConstraints(MotionProfileConstraints(parameters.profile_max_velocity, parameters.profile_max_acceleration));
            }

            Twist2d PathFollower::update(const units::Time& t, const RigidTransform2d& pose, const units::Distance& displacement, const units::Velocity& velocity)
            {
                if(!m_steering_controller.finished())
                {
                    const AdaptivePurePursuitController::Command steering_command = m_steering_controller.update(pose);
                    m_cross_track_error = steering_command.cross_track_error;
                    m_last_steering_delta = steering_command.delta;
                    m_velocity_controller.setGoalAndConstraints(MotionProfileGoal(displacement + steering_command.delta.dx(),
                    units::abs(steering_command.end_velocity), MotionProfileGoal::CompletionBehavior::kViolateMaxAccel, m_goal_position_tolerance, m_goal_velocity_tolerance),
                    MotionProfileConstraints(units::min(m_max_profile_velocity, steering_command.max_velocity), m_max_profile_acceleration));
                    if(steering_command.remaining_path_length < m_stop_steering_distance)
                    {
                        m_done_steering = true;
                    }
                }

                const units::Velocity velocity_command = m_velocity_controller.update(MotionState(t, displacement, velocity, 0.0), t);
                m_along_track_error = m_velocity_controller.getPositionError();
                const double curvature = m_last_steering_delta.dtheta()() / m_last_steering_delta.dx()();
                units::Angle dtheta = m_last_steering_delta.dtheta();
                if(!std::isnan(curvature) && std::abs(curvature) < 1e6)
                {
                    // Regenerate angular velocity command from adjusted curvature.
                    const units::Velocity velocity_setpoint = units::abs(m_velocity_controller.setpoint().velocity());
                    dtheta = m_last_steering_delta.dx()() * curvature * (1.0 + m_inertia_gain * velocity_setpoint());
                }
                const double scale = velocity_command() / m_last_steering_delta.dx()();

                return Twist2d(m_last_steering_delta.dx() * scale, 0.0, dtheta * scale);
            }

            units::Distance PathFollower::crossTrackError() const
            {
                return m_cross_track_error;
            }

            units::Distance PathFollower::alongTrackError() const
            {
                return m_along_track_error;
            }

            bool PathFollower::finished() const
            {
                return (m_steering_controller.finished() && m_velocity_controller.finished() && m_velocity_controller.onTarget()) || m_override_finished;
            }

            void PathFollower::forceFinish()
            {
                m_override_finished = true;
            }

            bool PathFollower::hasPassedMarker(const std::string& marker)
            {
                return m_steering_controller.hasPassedMarker(marker);
            }
        }
    }
}