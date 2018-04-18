#ifndef FOLLOW_PATH_HPP
#define FOLLOW_PATH_HPP

#include <path_follower/path_follower.hpp>
#include <drivetrains/drivetrain.hpp>
#include <framework/action.hpp>
#include <pose/robot_pose.hpp>


#include <json.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class FollowPath : public framework::Action
            {
            public:
                FollowPath(const nlohmann::json& config, std::shared_ptr<drivetrains::Drivetrain> drivetrain, pathfollower::RigidTransform2d* pose = nullptr);

                /**
                * Returns whether or not the action has finished execution.
                */
                virtual bool isFinished() override;

                /**
                 * Iteratively called until {@see Action#isFinished()} returns true
                 */
                virtual void update(nlohmann::json& state) override;

                /**
                 * Run once before the main code
                 */
                virtual void setup(nlohmann::json& state) override;

                /**
                 * Run once after finished
                 */
                virtual void teardown(nlohmann::json& state) override;

            private:
                std::unique_ptr<pose::RobotPose> m_pose;
                std::unique_ptr<pathfollower::PathFollower> m_path_follower;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;

                pathfollower::RigidTransform2d m_starting_point;
                units::Distance m_initial_left;
                units::Distance m_initial_right;
            };
        }
    }
}

#endif // FOLLOW_PATH_HPP
