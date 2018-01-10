#ifndef PATH_FOLLOWER_ACTION_HPP
#define PATH_FOLLOWER_ACTION_HPP

#include <memory>
#include <chrono>

#include "drivetrain.hpp"
#include <units.hpp>

namespace rip
{
    namespace navigation
    {
        /**
         * An action for following a path
         */
        class PathFollowerAction : public Action
        {
        public:
            /**
             * Constructor
             *
             * @param filepath The path to the file containing the configuration for the path to follow
             * @param drivetrain The controller for the drive train
             */
            PathFollowerAction(const std::string& filepath, std::shared_ptr<Drivetrain> drivetrain)
                : m_filepath(filepath)
                , m_drivetrain(drivetrain)
            {
            }

            /**
             * Initialization
             *
             * @note separate from constructor so an exception can be thrown if the config file does not
             * exist
             */
            void init()
            {
                cppfs::FileHandler fh = fs::open(m_filepath);

                if (fh.exists() && fh.isFile())
                {
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
                    m_follower = PathFollower(pathplanner, m_drivetrain);

                }
                else
                {
                    // throw exception
                }
            }

            /**
             * Returns whether the action has been finished
             */
            virtual bool isFinished() override
            {
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                units::Time t = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() * units::ms;
                return m_follower.isFinished(t);
            }

            /**
             * Sets up the action by recording the start time
             */
            virtual void setup() override
            {
                m_start_time = std::chrono::system_clock::now();
            }

            /**
             * Updates the action upon each cycle
             */
            virtual void update() override
            {
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                units::Time t = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() * units::ms;
                m_follower.update(t);
            }

            /**
             * Saves state in case of a crash
             */
            virtual nlohmann::json save() const override
            {
                nlohmann::json j;

                // todo

                return j;
            }

            /**
             * Restores action from a crash
             */
            virtual void restore(const nlohmann::json& config)
            {
                // todo
            }

            /**
             * Tears down the function
             */
            virtual void teardown() override
            {
            }


        private:
            std::string m_filepath;
            std::shared_ptr<Drivetrain> m_drivetrain;
            PathFollower m_follower;
            std::chrono::time_point<std::chrono::system_clock> m_start_time;
            units::Time m_drive_time;
        };
    }
}

#endif // PATH_FOLLOWER_ACTION_HPP