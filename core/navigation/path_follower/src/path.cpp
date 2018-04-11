#include "path_follower/path.hpp"

#include "misc/constants.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            void Path::addSegment(const PathSegment& segment)
            {
                m_segments.push_back(segment);
            }

            void Path::extrapolateLast()
            {
                PathSegment& last = m_segments.back();
                last.extrapolateLookahead(true);
            }

            Translation2d Path::endPosition() const
            {
                return m_segments.back().end();
            }

            MotionState Path::lastMotionState() const
            {
                if (m_segments.size() > 0)
                {
                    MotionState end_state = m_segments.back().endState();
                    return MotionState(0.0, 0.0, end_state.velocity(), end_state.acceleration());
                }
                else
                {
                    return MotionState(0.0, 0.0, 0.0, 0.0);
                }
            }

            units::Distance Path::getSegmentRemainingDistance(const Translation2d& position) const
            {
                const PathSegment& current_segment = m_segments.front();
                return current_segment.remainingDistance(current_segment.closestPoint(position));
            }

            units::Distance Path::segmentLength() const
            {
                const PathSegment& current_segment = m_segments.front();
                return current_segment.length();
            }

            Path::TargetPointReport Path::targetPoint(const Translation2d& robot, const Lookahead& lookahead)
            {
                TargetPointReport rv;
                PathSegment& current_segment = m_segments.front();
                rv.closest_point = current_segment.closestPoint(robot);
                rv.closest_point_distance = Translation2d(robot, rv.closest_point).norm();
                rv.remaining_segment_distance = current_segment.remainingDistance(rv.closest_point);
                rv.remaining_path_distance = rv.remaining_segment_distance;
                for (size_t i = 1; i < m_segments.size(); i++)
                {
                    rv.remaining_path_distance += m_segments[i].length();
                }
                rv.closest_point_speed = current_segment.speedByDistance(current_segment.length() - rv.remaining_segment_distance);
                units::Distance lookahead_distance = lookahead.getLookaheadForSpeed(rv.closest_point_speed) + rv.closest_point_distance;
                if (rv.remaining_segment_distance < lookahead_distance && m_segments.size() > 1)
                {
                    lookahead_distance -= rv.remaining_segment_distance;
                    for (size_t i = 1; i < m_segments.size(); ++i)
                    {
                        current_segment = m_segments[i];
                        const units::Distance length = current_segment.length();
                        if (length < lookahead_distance && i < m_segments.size() - 1)
                        {
                            lookahead_distance -= length;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    lookahead_distance += (current_segment.length() - rv.remaining_segment_distance);
                }
                rv.max_speed = current_segment.maxSpeed();
                rv.lookahead_point = current_segment.pointByDistance(lookahead_distance);
                rv.lookahead_point_speed = current_segment.speedByDistance(lookahead_distance);
                checkSegmentFinished(rv.closest_point);
                return rv;
            }

            units::Velocity Path::speed(const Translation2d& position) const
            {
                const PathSegment& current_segment = m_segments.front();
                return current_segment.speedByClosestPoint(position);
            }

            void Path::checkSegmentFinished(const Translation2d& position)
            {
                PathSegment& current_segment = m_segments.front();
                units::Distance remaining_distance = current_segment.remainingDistance(current_segment.closestPoint(position));
                if (remaining_distance < misc::constants::get<units::Distance>(misc::constants::kSegmentCompletionTolerance))
                {
                    removeCurrentSegment();
                }
            }

            void Path::removeCurrentSegment()
            {
                m_previous = m_segments.front();
                m_segments.erase(m_segments.begin());
                std::string marker = m_previous.marker();
                if (!marker.empty())
                {
                    m_markers_crossed.insert(marker);
                }
            }

            void Path::verifySpeeds()
            {
                units::Velocity max_start_speed = 0.0;
                units::Velocity start_speeds[m_segments.size() + 1];
                start_speeds[m_segments.size()] = 0.0;
                for (size_t i = m_segments.size() - 1; i >= 0; --i)
                {
                    PathSegment& segment = m_segments[i];
                    max_start_speed += units::sqrt(max_start_speed * max_start_speed + 2 * misc::constants::get<units::Acceleration>(misc::constants::kMaxAcceleration) * segment.length());
                    start_speeds[i] = segment.startState().velocity();
                    if (start_speeds[i] > max_start_speed)
                    {
                        start_speeds[i] = max_start_speed;
                    }
                    max_start_speed = start_speeds[i];
                }
                for (size_t i = 0; i < m_segments.size(); ++i)
                {
                    PathSegment& segment = m_segments[i];
                    units::Velocity end_speed = start_speeds[i + 1];
                    MotionState start_state = (i > 0) ? m_segments[i - 1].endState() : MotionState(0, 0, 0, 0);
                    start_state = MotionState(0, 0, start_state.velocity(), start_state.velocity()());
                    segment.createMotionProfiler(start_state, end_speed);
                }
            }

            bool Path::hasPassedMarker(const std::string& marker)
            {
                return m_markers_crossed.find(marker) != m_markers_crossed.end();
            }

            std::string Path::toString() const
            {
                std::string str = "";
                for (const PathSegment& s : m_segments)
                {
                    str += s.toString() + "\n";
                }
                return str;
            }

            std::ostream& operator<<(std::ostream& os, const Path& path)
            {
                os << path.toString() << std::endl;
                return os;
            }
        }
    }
}
