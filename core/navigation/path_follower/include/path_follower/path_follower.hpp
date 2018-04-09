#ifndef PATH_FOLLOWER_HPP
#define PATH_FOLLOWER_HPP

#include "path_follower/lookahead.hpp"
#include "path_follower/path.hpp"
#include "path_follower/twist_2d.hpp"
#include "path_follower/rigid_transform_2d.hpp"
#include "path_follower/adaptive_pure_pursuit_controller.hpp"
#include "path_follower/profile_follower.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class PathFollower
            {
            public:
                struct Parameters
                {
                    Lookahead lookahead;
                    double inertia_gain;
                    double profile_kp;
                    double profile_ki;
                    double profile_kv;
                    double profile_kffv;
                    double profile_kffa;
                    units::Velocity profile_max_velocity;
                    units::Acceleration profile_max_acceleration;
                    units::Distance goal_position_tolerance;
                    units::Velocity goal_velocity_tolerance;
                    units::Distance stop_steering_distance;
                };

                PathFollower(const Path& path, bool reversed, const Parameters& parameters);
                Twist2d update(const units::Time& t, const RigidTransform2d& pose, const units::Distance& displacement, const units::Velocity& velocity);
                units::Distance crossTrackError() const;
                units::Distance alongTrackError() const;
                bool finished() const;
                void forceFinish();
                bool hasPassedMarker(const std::string& marker);

            private:
                AdaptivePurePursuitController m_steering_controller;
                Twist2d m_last_steering_delta;
                ProfileFollower m_velocity_controller;
                double m_inertia_gain;
                bool m_override_finished;
                bool m_done_steering;
                units::Velocity m_max_profile_velocity;
                units::Acceleration m_max_profile_acceleration;
                units::Distance m_goal_position_tolerance;
                units::Velocity m_goal_velocity_tolerance;
                units::Distance m_stop_steering_distance;
                units::Distance m_cross_track_error;
                units::Distance m_along_track_error;
            };
        }
    }
}

#endif //PATH_FOLLOWER_HPP
