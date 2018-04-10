#ifndef PATH_SEGMENT_HPP
#define PATH_SEGMENT_HPP

#include <units/units.hpp>

#include "path_follower/motion_state.hpp"
#include "path_follower/motion_profile.hpp"
#include "path_follower/translation_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * Class representing a segment of the robot's autonomous path.
             */
            class PathSegment
            {
            public:
                /**
                 * @note Don't use this constructor. This is solely for vectors
                 */
                PathSegment() = default;

                /**
                 * Constructor for a linear segment
                 *
                 * @param x1
                 *            start x
                 * @param y1
                 *            start y
                 * @param x2
                 *            end x
                 * @param y2
                 *            end y
                 * @param maxSpeed
                 *            maximum speed allowed on the segment
                 */
                PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed);

                PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed, const std::string& marker);

                /**
                 * Constructor for an arc segment
                 *
                 * @param x1
                 *            start x
                 * @param y1
                 *            start y
                 * @param x2
                 *            end x
                 * @param y2
                 *            end y
                 * @param cx
                 *            center x
                 * @param cy
                 *            center y
                 * @param maxSpeed
                 *            maximum speed allowed on the segment
                 */
                PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Distance& cx, const units::Distance& cy, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed);

                PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Distance& cx, const units::Distance& cy, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed, const std::string& marker);

                units::Velocity maxSpeed() const;

                void createMotionProfiler(const MotionState& start_state, const units::Velocity& end_speed);

                /**
                 * @return starting point of the segment
                 */
                Translation2d start() const;

                /**
                 * @return end point of the segment
                 */
                Translation2d end() const;

                /**
                 * @return the total length of the segment
                 */
                units::Distance length() const;

                /**
                 * Set whether or not to extrapolate the lookahead point. Should only be true for the last segment in the path
                 *
                 * @param val
                 */
                void extrapolateLookahead(bool value);

                /**
                 * Gets the point on the segment closest to the robot
                 *
                 * @param position
                 *            the current position of the robot
                 * @return the point on the segment closest to the robot
                 */
                Translation2d closestPoint(const Translation2d position) const;

                /**
                 * Calculates the point on the segment <code>distance</code> distance from the starting point along the segment.
                 *
                 * @param distance
                 *            distance from the starting point
                 * @return point on the segment <code>distance</code> distance from the starting point
                 */
                Translation2d pointByDistance(units::Distance distance) const;

                /**
                 * Gets the remaining distance left on the segment from point <code>point</code>
                 *
                 * @param point
                 *            result of <code>getClosestPoint()</code>
                 * @return distance remaining
                 */
                units::Distance remainingDistance(const Translation2d& position) const;

                units::Distance distanceTravelled(const Translation2d& position) const;

                units::Velocity speedByDistance(units::Distance distance) const;

                units::Velocity speedByClosestPoint(const Translation2d& position) const;

                MotionState endState() const;
                MotionState startState() const;

                std::string marker() const;

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream&, const PathSegment&);
            private:
                Translation2d m_start;
                Translation2d m_end;
                Translation2d m_center;
                Translation2d m_delta_start;
                Translation2d m_delta_end;
                units::Velocity m_max_speed;
                bool m_line;
                MotionProfile m_speed_controller;
                bool m_extrapolate_lookahead;
                std::string m_marker;
            };

            std::ostream& operator<<(std::ostream&, const PathSegment&);
        }
    }
}

#endif //PATH_SEGMENT_HPP