#ifndef DIFFERENTIAL_DRIVE_KINEMATICS_HPP
#define DIFFERENTIAL_DRIVE_KINEMATICS_HPP

#include <units/units.hpp>

#include "path_follower/twist_2d.hpp"
#include "path_follower/rigid_transform_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class DifferentialDriveKinematics
            {
            public:
                /**
                 * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
                 * motion)
                 */
                static Twist2d forwardKinematics(const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta);

                /**
                 * Forward kinematics using encoders and explicitly measured rotation (ex. from gyro)
                 */
                static Twist2d forwardKinematics(const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const units::Angle& rotation_delta);

                /**
                 * For convenience, forward kinematic with an absolute rotation and previous rotation.
                 */
                static Twist2d forwardKinematics(const Rotation2d& previous_heading, const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const Rotation2d& current_heading);

                /**
                 * Append the result of forward kinematics to a previous pose.
                 */
                static RigidTransform2d integrateForwardKinematics(const RigidTransform2d& current_pose, const units::Distance& left_wheel_delta, const units::Distance& right_wheel_delta, const Rotation2d& current_heading);

                /**
                 * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
                 */
                static RigidTransform2d integrateForwardKinematics(const RigidTransform2d& current_post, const Twist2d& forward_kinematics);

                /**
                 * Class that contains left and right wheel velocities
                 */
                class DriveVelocity
                {
                public:
                    DriveVelocity(const units::Velocity& left, const units::Velocity& right);

                    units::Velocity left() const;
                    units::Velocity right() const;
                private:
                    units::Velocity m_left;
                    units::Velocity m_right;
                };

                /**
                 * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
                 */
                static DriveVelocity inverseKinematics(const Twist2d& delta);
            };
        }
    }
}

#endif // DIFFERENTIAL_DRIVE_KINEMATICS_HPP
