#include "path_follower/path_segment.hpp"
#include "path_follower/motion_profile_constraints.hpp"
#include "path_follower/motion_profile_goal.hpp"
#include "path_follower/motion_profile_generator.hpp"

#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            PathSegment::PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed)
                : m_start(x1, y1), m_end(x2, y2), m_max_speed(max_speed), m_extrapolate_lookahead(false), m_line(true)
            {
                m_delta_start = Translation2d(m_start, m_end);
                createMotionProfiler(start_state, end_speed);
            }

            PathSegment::PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed, const std::string& marker)
                : m_start(x1, y1), m_end(x2, y2), m_max_speed(max_speed), m_extrapolate_lookahead(false), m_line(true), m_marker(marker)
            {
                m_delta_start = Translation2d(m_start, m_end);
                createMotionProfiler(start_state, end_speed);
            }

            PathSegment::PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Distance& cx, const units::Distance& cy, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed)
                : m_start(x1, y1)
                , m_end(x2, y2)
                , m_center(cx, cy)
                , m_max_speed(max_speed)
                , m_extrapolate_lookahead(false)
                , m_line(false)
            {
                m_delta_start = Translation2d(m_center, m_start);
                m_delta_end = Translation2d(m_center, m_end);

                createMotionProfiler(start_state, end_speed);
            }

            PathSegment::PathSegment(const units::Distance& x1, const units::Distance& y1, const units::Distance& x2, const units::Distance& y2, const units::Distance& cx, const units::Distance& cy, const units::Velocity& max_speed, const MotionState& start_state, const units::Velocity& end_speed, const std::string& marker)
                : m_start(x1, y1)
                , m_end(x2, y2)
                , m_center(cx, cy)
                , m_max_speed(max_speed)
                , m_extrapolate_lookahead(false)
                , m_line(false)
                , m_marker(marker)
            {
                m_delta_start = Translation2d(m_center, m_start);
                m_delta_end = Translation2d(m_center, m_end);

                createMotionProfiler(start_state, end_speed);
            }

            units::Velocity PathSegment::maxSpeed() const
            {
                return m_max_speed;
            }

            void PathSegment::createMotionProfiler(const MotionState& start_state, const units::Velocity& end_speed)
            {
                MotionProfileConstraints motion_constraints(m_max_speed, misc::constants::get<units::Acceleration>(misc::constants::kMaxAcceleration));
                MotionProfileGoal goal_state(length(), end_speed);
                m_speed_controller = MotionProfileGenerator::generateProfile(motion_constraints, goal_state, start_state);
            }

            Translation2d PathSegment::start() const
            {
                return m_start;
            }

            Translation2d PathSegment::end() const
            {
                return m_end;
            }

            units::Distance PathSegment::length() const
            {
                if (m_line)
                {
                    return m_delta_start.norm();
                }
                else
                {
                    return m_delta_start.norm() * Translation2d::getAngle(m_delta_start, m_delta_end).to(units::rad);
                }
            }

            void PathSegment::extrapolateLookahead(bool value)
            {
                m_extrapolate_lookahead = value;
            }

            Translation2d PathSegment::closestPoint(const Translation2d position) const
            {
                if (m_line)
                {
                    Translation2d delta(m_start, m_end);
                    double u = (((position.x() - m_start.x()) * delta.x() + (position.y() - start().y()) * delta.y()) / (delta.x() * delta.x() + delta.y() * delta.y()))();
                    if (u >= 0.0 && u <= 1)
                    {
                        return Translation2d(m_start.x() + u * delta.x(), m_start.y() + u * delta.y());
                    }
                    return u < 0.0 ? m_start : m_end;
                }
                else
                {
                    Translation2d delta_position(m_center, position);
                    delta_position = delta_position.scale((m_delta_start.norm() / delta_position.norm())());
                    if (Translation2d::cross(delta_position, m_delta_start) * Translation2d::cross(delta_position, m_delta_end) < 0)
                    {
                        return m_center.translateBy(delta_position);
                    }
                    else
                    {
                        Translation2d start_dist(position, m_start);
                        Translation2d end_dist(position, m_end);
                        return (end_dist.norm() < start_dist.norm()) ? m_end : m_start;
                    }
                }
            }

            Translation2d PathSegment::pointByDistance(units::Distance distance) const
            {
                units::Distance length = length();
                if (!m_extrapolate_lookahead && distance > length)
                {
                    distance = length;
                }
                if (m_line)
                {
                    return m_start.translateBy(m_delta_start.scale((distance / length)()));
                }
                else
                {
                    units::Angle delta_angle = Translation2d::getAngle(m_delta_start, m_delta_end) * ((Translation2d::cross(m_delta_start, m_delta_end) >= 0.0) ? 1 : -1);
                    delta_angle *= (distance / length)();
                    Translation2d t = m_delta_start.rotateBy(delta_angle);
                    return m_center.translateBy(t);
                }
            }

            units::Distance PathSegment::remainingDistance(const Translation2d& position) const
            {
                if (m_line)
                {
                    return Translation2d(m_end, position).norm();
                }
                else
                {
                    Translation2d delta_position(m_center, position);
                    units::Angle angle = Translation2d::getAngle(m_delta_end, delta_position);
                    units::Angle total_angle = Translation2d::getAngle(m_delta_start, m_delta_end);
                    return (angle / total_angle)() * length();
                }
            }

            units::Distance PathSegment::distanceTravelled(const Translation2d& position) const
            {
                Translation2d path_position = closestPoint(position);
                return length() - remainingDistance(path_position);
            }

            units::Velocity PathSegment::speedByDistance(units::Distance distance) const
            {
                if (distance < m_speed_controller.startPosition())
                {
                    distance = m_speed_controller.startPosition();
                }
                else if (distance > m_speed_controller.endPosition())
                {
                    distance = m_speed_controller.endPosition();
                }
                nonstd::optional<MotionState> state = m_speed_controller.firstStateByPosition(distance);
                if (state)
                {
                    return state.value().velocity();
                }
                else
                {
                    misc::Logger::getInstance()->error("Velocity does not exist at that position!");
                    return 0.0;
                }
            }

            units::Velocity PathSegment::speedByClosestPoint(const Translation2d& position) const
            {
                return speedByDistance(distanceTravelled(position));
            }

            MotionState PathSegment::endState() const
            {
                return m_speed_controller.endState();
            }

            MotionState PathSegment::startState() const
            {
                return m_speed_controller.startState();
            }

            std::string PathSegment::marker() const
            {
                return m_marker;
            }

            std::string PathSegment::toString() const
            {
                if (m_line)
                {
                    return fmt::format("(start: {0:s}, end: {1:s}, speed: {2:0.2f})", m_start.toString(), m_end.toString(), m_max_speed.to(units::in / units::s));
                }
                else
                {
                    return fmt::format("(start: {0:s}, end: {1:s}, center: {2:s}, speed: {3:0.2f})", m_start.toString(), m_end.toString(), m_center.toString(), m_max_speed.to(units::in / units::s));

                }
            }

            std::ostream& operator<<(std::ostream& os, const PathSegment& ps)
            {
                os << ps.toString() << std::endl;
                return os;
            }
        }
    }
}
