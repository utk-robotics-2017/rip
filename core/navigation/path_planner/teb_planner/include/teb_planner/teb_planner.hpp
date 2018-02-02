#ifndef TEB_PLANNER_HPP
#define TEB_PLANNER_HPP

#include <memory>
#include <vector>

#include <teb_planner/teb_config.hpp>
#include <teb_planner/obstacles.hpp>
#include <teb_planner/homotopy_class_planner.hpp>
#include <teb_planner/robot_footprint_model.hpp>
#include <teb_planner/fake_ros_msgs.hpp>
#include <teb_planner/trajectory_point.hpp>
#include <teb_planner/pose.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            class TebPlanner
            {
            public:
                TebPlanner(std::shared_ptr<TebConfig> config, std::vector< std::shared_ptr<Obstacle> > obstacles, std::shared_ptr<BaseRobotFootprintModel> robot)
                    : m_config(config)
                    , m_obstacles(obstacles)
                    , m_robot(robot)
                    , m_planner(new HomotopyClassPlanner(config, &obstacles, robot))
                {}

                void plan(const Pose& start, const Pose& goal)
                {
                    m_planner->plan(start, goal);
                }

                std::vector< TrajectoryPoint > getTrajectory()
                {
                    std::vector< fakeros::TrajectoryPointMsg > msgs;
                    m_planner->bestTeb()->getFullTrajectory(msgs);

                    std::vector< TrajectoryPoint > ret;
                    for (const fakeros::TrajectoryPointMsg& msg : msgs)
                    {
                        ret.emplace_back(msg);
                    }

                    return ret;
                }

            private:
                std::shared_ptr<TebConfig> m_config;
                std::vector< std::shared_ptr<Obstacle> > m_obstacles;
                std::shared_ptr<BaseRobotFootprintModel> m_robot;
                std::shared_ptr<HomotopyClassPlanner> m_planner;
            };
        }
    }
}

#endif //TEB_PLANNER_HPP
