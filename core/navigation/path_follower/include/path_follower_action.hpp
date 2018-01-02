#ifndef PATH_FOLLOWER_ACTION_HPP
#define PATH_FOLLOWER_ACTION_HPP

#include <memory>

#include "drivetrain.hpp"

namespace rip
{
    namespace navigation
    {
        class PathFollowerAction : public Action
        {
        public:
            PathFollowerAction(const std::string& filepath, std::shared_ptr<Drivetrain> drivetrain)
            {
                cppfs::FileHandler fh = fs::open(filepath);

                if (fh.exists() && fh.isFile())
                {
                    // exception / init function ?
                    std::unique_ptr<std::istream> in = fh.createInputStream();
                    nlohmann::json j;
                    (*in) >> j;

                    units::Distance width = j["robot_width"];
                    units::Velocity max_velocity = j["max_velocity"];
                    units::Acceleration max_acceleration = j["max_acceleration"];
                    units::Jerk max_jerk = j["max_jerk"];
                    std::vector<Waypoint> waypoints;
                    for (nlohmann::json j_waypoint : j["waypoints"])
                    {
                        Waypoint waypoint = j_waypoint;
                        waypoints.push_back(waypoint);
                    }

                    PathPlanner pathplanner(waypoint);
                    pathplanner.setRobotConfig(width, max_velocity, max_acceleration, max_jerk);
                    m_follower = PathFollower(pathplanner, drivetrain);

                }
            }

            virtual bool isFinished() override
            {
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                units::Time t = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() * units::ms;
                return m_follower.isFinished(t);
            }

            virtual void setup() override
            {
                m_start_time = std::chrono::system_clock::now();
            }

            virtual void update() override
            {
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                units::Time t = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() * units::ms;
                m_follower.update(t);
            }

            virtual void teardown() override
            {
            }


        private:
            PathFollower m_follower;
            std::chrono::time_point<std::chrono::system_clock> m_start_time;
            units::Time m_drive_time;
        };
    }
}

#endif // PATH_FOLLOWER_ACTION_HPP