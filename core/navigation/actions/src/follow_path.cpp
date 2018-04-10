#include "navigation_actions/follow_path.hpp"

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include "navigation_actions/exceptions.hpp"

#include "path_follower/path_builder.hpp"
#include "path_follower/differential_drive_kinematics.hpp"

#include "motor_controllers/motor_dynamics.hpp"

#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {

            FollowPath::FollowPath(const nlohmann::json& config, std::shared_ptr<drivetrains::Drivetrain> drivetrain, pathfollower::RigidTransform2d* pose)
                : framework::Action("FollowPath")
                , m_drivetrain(drivetrain)
            {
                std::string filepath = config["waypoint_filepath"];

                cppfs::FileHandle waypoints_file = cppfs::fs::open(filepath);

                if (!waypoints_file.exists() || !waypoints_file.isFile())
                {
                    throw FileNotFound("Waypoints file {} does not exist", filepath);
                }

                nlohmann::json waypoints_json = nlohmann::json::parse(waypoints_file.readFile());
                std::vector<pathfollower::Waypoint> waypoints;
                for(const nlohmann::json& waypoint : waypoints_json)
                {
                    waypoints.push_back(waypoint.get<pathfollower::Waypoint>());
                }
                pathfollower::Path path = pathfollower::PathBuilder::buildPathFromWaypoints(waypoints);

                pathfollower::PathFollower::Parameters parameters = config["parameters"];

                nlohmann::json pose_parameters = config["pose"];

                if(pose)
                {
                    m_starting_point = pathfollower::RigidTransform2d(*pose);
                }

                m_pose = std::unique_ptr<pose::RobotPose>(new pose::RobotPose(misc::constants::getInstance()->get<units::Distance>(misc::constants::kWheelbase), pose_parameters["alpha"], pose_parameters["beta"], pose_parameters["kappa"]));

                m_path_follower = std::unique_ptr<pathfollower::PathFollower>(new pathfollower::PathFollower(path, config["reversed"].get<bool>(), parameters));
            }

            bool FollowPath::isFinished()
            {
                return m_path_follower->finished();
            }

            void FollowPath::update(nlohmann::json& state)
            {
                units::Time now = units::now();

                units::Distance left_distance = m_drivetrain->readEncoder(drivetrains::Drivetrain::Motor::kLeft) - m_initial_left;
                units::Distance right_distance = m_drivetrain->readEncoder(drivetrains::Drivetrain::Motor::kRight) - m_initial_right;
                units::Velocity left_velocity = m_drivetrain->readEncoderVelocity(drivetrains::Drivetrain::Motor::kLeft);
                units::Velocity right_velocity = m_drivetrain->readEncoderVelocity(drivetrains::Drivetrain::Motor::kRight);
                m_pose->updateOdometry(now, left_velocity, right_velocity);
                units::Angle heading = m_drivetrain->readGyro();
                m_pose->updateImu(now, heading);

                misc::Logger::getInstance()->debug("t: {} s, ld: {} in, rd: {} in, lv: {} in/s, rv: {} in/s, h: {} deg", now.to(units::s), left_distance.to(units::in), right_distance.to(units::in), left_velocity.to(units::in/units::s), right_velocity.to(units::in/units::s), heading.to(units::deg));

                pathfollower::RigidTransform2d pose = m_starting_point + m_pose->pose();

                pathfollower::Twist2d twist = m_path_follower->update(now, pose, (left_distance + right_distance) / 2.0, (left_velocity + right_velocity) / 2.0);

                pathfollower::DifferentialDriveKinematics::DriveVelocity velocity = pathfollower::DifferentialDriveKinematics::inverseKinematics(twist);
                misc::Logger::getInstance()->debug("t: {} s, l: {} in/s, r: {} in/s", now.to(units::s), velocity.left().to(units::in/units::s), velocity.right().to(units::in/units::s));

                motorcontrollers::MotorDynamics left;
                left.setSpeed(velocity.left());
                motorcontrollers::MotorDynamics right;
                right.setSpeed(velocity.right());

                m_drivetrain->drive(left, right);
            }

            void FollowPath::setup(nlohmann::json& state)
            {
                m_initial_left = m_drivetrain->readEncoder(drivetrains::Drivetrain::Motor::kLeft);
                m_initial_right = m_drivetrain->readEncoder(drivetrains::Drivetrain::Motor::kRight);
            }

            void FollowPath::teardown(nlohmann::json& state)
            {

            }

        }
    }
}
