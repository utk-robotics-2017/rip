#ifndef PATH_HPP
#define PATH_HPP

#include <ostream>
#include <vector>
#include <string>
#include <set>

#include <units/units.hpp>

#include "path_follower/path_segment.hpp"
#include "path_follower/lookahead.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * Class representing the robot's autonomous path.
             *
             * Field Coordinate System: Uses a right hand coordinate system. Positive x is right, positive y is up, and the origin
             * is at the bottom left corner of the field. For angles, 0 degrees is facing right (1, 0) and angles increase as you
             * turn counter clockwise.
             */
            class Path
            {
            public:
                Path() = default;

                /**
                 * add a segment to the Path
                 *
                 * @param segment
                 *            the segment to add
                 */
                void addSegment(const PathSegment& segment);

                void extrapolateLast();

                Translation2d endPosition() const;

                /**
                 * @return the last MotionState in the path
                 */
                MotionState lastMotionState() const;

                /**
                 * get the remaining distance left for the robot to travel on the current segment
                 *
                 * @param robotPos
                 *            robot position
                 * @return remaining distance on current segment
                 */
                units::Distance getSegmentRemainingDistance(const Translation2d& position) const;

                /**
                 * @return the length of the current segment
                 */
                units::Distance segmentLength() const;

                struct TargetPointReport
                {
                    Translation2d closest_point;
                    units::Distance closest_point_distance;
                    units::Velocity closest_point_speed;
                    Translation2d lookahead_point;
                    units::Velocity max_speed;
                    units::Velocity lookahead_point_speed;
                    units::Distance remaining_segment_distance;
                    units::Distance remaining_path_distance;
                };

                /**
                 * Gives the position of the lookahead point (and removes any segments prior to this point).
                 *
                 * @param robot
                 *            Translation of the current robot pose.
                 * @return report containing everything we might want to know about the target point.
                 */
                TargetPointReport targetPoint(const Translation2d& robot, const Lookahead& lookahead);

                /**
                 * Gives the speed the robot should be traveling at the given position
                 *
                 * @param robotPos
                 *            position of the robot
                 * @return speed robot should be traveling
                 */
                units::Velocity speed(const Translation2d& position) const;

                /**
                 * Checks if the robot has finished traveling along the current segment then removes it from the path if it has
                 *
                 * @param robotPos
                 *            robot position
                 */
                void checkSegmentFinished(const Translation2d& position);

                void removeCurrentSegment();

                /**
                 * Ensures that all speeds in the path are attainable and robot can slow down in time
                 */
                void verifySpeeds();

                bool hasPassedMarker(const std::string& marker);

                std::string toString() const;

                friend std::ostream& operator<<(std::ostream& os, const Path& path);

            private:
                std::vector<PathSegment> m_segments;
                PathSegment m_previous;
                std::set<std::string> m_markers_crossed;
            };

            std::ostream& operator<<(std::ostream& os, const Path& path);
        }
    }
}

#endif //PATH_HPP
