#include "path_follower/adaptive_pure_pursuit_controller.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            AdaptivePurePursuitController::AdaptivePurePursuitController(const Path& path, bool reversed, const Lookahead& lookahead)
                    : m_path(path), m_reversed(reversed), m_lookahead(lookahead)
            {}

            template <typename T>
            int signum(T a)
            {
                return (0.0 < a) - (a < 0.0);
            }

            AdaptivePurePursuitController::Command AdaptivePurePursuitController::update(RigidTransform2d pose)
            {
                if(m_reversed)
                {
                    pose = RigidTransform2d(pose.translation(), pose.rotation().rotateBy(Rotation2d(units::pi)));
                }

                const Path::TargetPointReport report = m_path.targetPoint(pose.translation(), m_lookahead);
                if(finished())
                {
                    // Stop
                    Command command;
                    command.delta = Twist2d();
                    command.cross_track_error = report.closest_point_distance;
                    command.max_velocity = report.max_speed;
                    command.end_velocity = 0.0;
                    command.lookahead_point = report.lookahead_point;
                    command.remaining_path_length = report.remaining_path_distance;
                    return command;
                }

                const Arc arc(pose, report.lookahead_point);
                double scale_factor = 1.0;
                // Ensure we don't overshoot the end of the path (once the lookahead speed drops to zero).
                if(report.lookahead_point_speed < 1e-6 && report.remaining_path_distance < arc.length())
                {
                    scale_factor = std::max<double>(0.0, (report.remaining_path_distance / arc.length())());
                    m_at_end_of_path = true;
                }
                else
                {
                    m_at_end_of_path = false;
                }
                if(m_reversed)
                {
                    scale_factor *= -1;
                }

                Command command;
                command.delta = Twist2d(scale_factor * arc.length(),
                                        0.0,
                                        (arc.length() * direction(pose, report.lookahead_point) * std::abs(scale_factor) / arc.radius())() * units::rad);
                command.cross_track_error = report.closest_point_distance;
                command.max_velocity = report.max_speed;
                command.end_velocity = report.lookahead_point_speed * signum(scale_factor);
                command.lookahead_point = report.lookahead_point;
                command.remaining_path_length = report.remaining_path_distance;
                return command;
            }

            bool AdaptivePurePursuitController::hasPassedMarker(const std::string& marker)
            {
                return m_path.hasPassedMarker(marker);
            }

            AdaptivePurePursuitController::Arc::Arc(const RigidTransform2d& pose, const Translation2d& point)
            {
                m_center = AdaptivePurePursuitController::center(pose, point);
                m_radius = AdaptivePurePursuitController::radius(pose, point);
                m_length = AdaptivePurePursuitController::length(pose, point, m_center, m_radius);
            }

            Translation2d AdaptivePurePursuitController::Arc::center() const
            {
                return m_center;
            }

            units::Distance AdaptivePurePursuitController::Arc::radius() const
            {
                return m_radius;
            }

            units::Distance AdaptivePurePursuitController::Arc::length() const
            {
                return m_length;
            }

            Translation2d AdaptivePurePursuitController::center(const RigidTransform2d& pose, const Translation2d& point)
            {
                const Translation2d pose_to_point_halfway = pose.translation().interpolate(point, 0.5);
                const Rotation2d normal = pose.translation().inverse().translateBy(pose_to_point_halfway).direction().normal();
                const RigidTransform2d perpendicular_bisector(pose_to_point_halfway, normal);
                const RigidTransform2d normal_from_pose(pose.translation(), pose.rotation().normal());
                if(normal_from_pose.colinear(perpendicular_bisector.normal()))
                {
                    // Special case: center is poseToPointHalfway.
                    return pose_to_point_halfway;
                }
                return normal_from_pose.intersection(perpendicular_bisector);
            }

            units::Distance AdaptivePurePursuitController::radius(const RigidTransform2d& pose, const Translation2d& point)
            {
                const Translation2d center = AdaptivePurePursuitController::center(pose, point);
                return Translation2d(center, point).norm();
            }

            units::Distance AdaptivePurePursuitController::length(const RigidTransform2d& pose, const Translation2d& point)
            {
                const units::Distance radius = AdaptivePurePursuitController::radius(pose, point);
                const Translation2d center = AdaptivePurePursuitController::center(pose, point);
                return length(pose, point, center, radius);
            }

            units::Distance AdaptivePurePursuitController::length(const RigidTransform2d& pose, const Translation2d& point, const Translation2d& center, const units::Distance& radius)
            {
                if(radius < 1e6)
                {
                    const Translation2d center_to_point(center, point);
                    const Translation2d center_to_pose(center, pose.translation());
                    // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                    // check the sign of the cross-product between the normal vector and the vector from pose to point.
                    const bool behind = signum(Translation2d::cross(pose.rotation().normal().toTranslation(), Translation2d(pose.translation(), point))) > 0.0;
                    const Rotation2d angle = Translation2d::getAngle(center_to_pose, center_to_point);
                    return radius * (behind ? 2.0 * units::pi - units::abs(angle.angle()) : units::abs(angle.angle()))();
                }
                else
                {
                    return Translation2d(pose.translation(), point).norm();
                }
            }

            int AdaptivePurePursuitController::direction(const RigidTransform2d& pose, const Translation2d& point)
            {
                const Translation2d pose_to_point(pose.translation(), point);
                const Translation2d robot = pose.rotation().toTranslation();
                double cross = (robot.x() * pose_to_point.y() - robot.y() * pose_to_point.x())();
                return (cross < 0) ? 1 : -1;// if robot < pose turn left
            }

            bool AdaptivePurePursuitController::finished() const
            {
                return m_at_end_of_path;
            }
        }
    }
}