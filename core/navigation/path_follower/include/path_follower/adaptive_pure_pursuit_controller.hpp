#ifndef ADAPTIVE_PURE_PURSUIT_CONTROLLER_HPP
#define ADAPTIVE_PURE_PURSUIT_CONTROLLER_HPP

#include "path_follower/twist_2d.hpp"
#include "path_follower/translation_2d.hpp"
#include "path_follower/rigid_transform_2d.hpp"
#include "path_follower/path.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * Implements an adaptive pure pursuit controller. See:
             * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4 .pdf
             *
             * Basically, we find a spot on the path we'd like to follow and calculate the arc necessary to make us land on that
             * spot. The target spot is a specified distance ahead of us, and we look further ahead the greater our tracking error.
             * We also return the maximum speed we'd like to be going when we reach the target spot.
             */
            class AdaptivePurePursuitController
            {
            public:
                struct Command
                {
                    Twist2d delta;
                    units::Distance cross_track_error;
                    units::Velocity max_velocity;
                    units::Velocity end_velocity;
                    Translation2d lookahead_point;
                    units::Distance remaining_path_length;
                };

                AdaptivePurePursuitController(const Path& path, bool reversed, const Lookahead& lookahead);

                /**
                 * Gives the RigidTransform2d.Delta that the robot should take to follow the path
                 *
                 * @param pose
                 *            robot pose
                 * @return movement command for the robot to follow
                 */
                Command update(RigidTransform2d pose);

                bool hasPassedMarker(const std::string& marker);

                class Arc
                {
                public:
                    Arc(const RigidTransform2d& pose, const Translation2d& point);

                    Translation2d center() const;
                    units::Distance radius() const;
                    units::Distance length() const;

                private:
                    Translation2d m_center;
                    units::Distance m_radius;
                    units::Distance m_length;
                };

                /**
                 * Gives the center of the circle joining the lookahead point and robot pose
                 *
                 * @param pose
                 *            robot pose
                 * @param point
                 *            lookahead point
                 * @return center of the circle joining the lookahead point and robot pose
                 */
                static Translation2d center(const RigidTransform2d& pose, const Translation2d& point);

                /**
                 * Gives the radius of the circle joining the lookahead point and robot pose
                 *
                 * @param pose
                 *            robot pose
                 * @param point
                 *            lookahead point
                 * @return radius of the circle joining the lookahead point and robot pose
                 */
                static units::Distance radius(const RigidTransform2d& pose, const Translation2d& point);

                /**
                 * Gives the length of the arc joining the lookahead point and robot pose (assuming forward motion).
                 *
                 * @param pose
                 *            robot pose
                 * @param point
                 *            lookahead point
                 * @return the length of the arc joining the lookahead point and robot pose
                 */
                static units::Distance length(const RigidTransform2d& pose, const Translation2d& point);

                static units::Distance length(const RigidTransform2d& pose, const Translation2d& point, const Translation2d& center, const units::Distance& radius);

                /**
                 * Gives the direction the robot should turn to stay on the path
                 *
                 * @param pose
                 *            robot pose
                 * @param point
                 *            lookahead point
                 * @return the direction the robot should turn: -1 is left, +1 is right
                 */
                static int direction(const RigidTransform2d& pose, const Translation2d& point);

                bool finished() const;

            private:
                Path m_path;
                bool m_at_end_of_path;
                bool m_reversed;
                Lookahead m_lookahead;
            };
        }
    }
}

#endif //ADAPTIVE_PURE_PURSUIT_CONTROLLER_HPP
