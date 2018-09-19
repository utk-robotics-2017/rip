#include "path_follower/differential_drive_kinematics.hpp"

#include <misc/constants.hpp>
#include "path_follower/epsilon.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {

            Twist2d rip::navigation::pathfollower::DifferentialDriveKinematics::forwardKinematics(const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta)
            {
                units::Distance delta_v = (right_wheel_delta - left_wheel_delta) / 2.0 * misc::constants::get<double>(misc::constants::kTrackScrubFactor);
                units::Angle delta_rotation = delta_v() * 2.0 / misc::constants::get<units::Distance>(misc::constants::kWheelbase)();
                return DifferentialDriveKinematics::forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
            }

            Twist2d DifferentialDriveKinematics::forwardKinematics(const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const units::Angle& rotation_delta)
            {
                units::Distance dx = (left_wheel_delta + right_wheel_delta) / 2.0;
                return Twist2d(dx, 0.0, rotation_delta);
            }

            Twist2d DifferentialDriveKinematics::forwardKinematics(const Rotation2d& previous_heading, const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const Rotation2d& current_heading)
            {
                return DifferentialDriveKinematics::forwardKinematics(left_wheel_delta, right_wheel_delta, previous_heading.inverse().rotateBy(current_heading).angle());
            }

            RigidTransform2d DifferentialDriveKinematics::integrateForwardKinematics(const RigidTransform2d& current_pose, const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const Rotation2d& current_heading)
            {
                Twist2d with_gyro = forwardKinematics(current_pose.rotation(), left_wheel_delta, right_wheel_delta, current_heading);
                return DifferentialDriveKinematics::integrateForwardKinematics(current_pose, with_gyro);
            }

            RigidTransform2d DifferentialDriveKinematics::integrateForwardKinematics(const RigidTransform2d& current_post, const Twist2d& forward_kinematics)
            {
                return current_post.transformBy(RigidTransform2d::exp(forward_kinematics));
            }

            DifferentialDriveKinematics::DriveVelocity DifferentialDriveKinematics::inverseKinematics(const Twist2d& delta)
            {
                if(units::abs(delta.dtheta()) < k_epsilon)
                {
                    return DriveVelocity(delta.dx()(), delta.dx()());
                }

                double delta_v = misc::constants::get<units::Distance>(misc::constants::kWheelbase)() * delta.dtheta()() / (2 * misc::constants::get<double>(misc::constants::kTrackScrubFactor));
                return DriveVelocity(delta.dx()() - delta_v, delta.dx()() + delta_v);
            }

            DifferentialDriveKinematics::DriveVelocity::DriveVelocity(const units::Velocity& left, const units::Velocity& right)
            : m_left(left), m_right(right) {}

            units::Velocity DifferentialDriveKinematics::DriveVelocity::left() const
            {
                return m_left;
            }

            units::Velocity DifferentialDriveKinematics::DriveVelocity::right() const
            {
                return m_right;
            }

        }
    }
}
