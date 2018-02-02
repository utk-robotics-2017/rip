#ifndef COMPUTE_THREAD_HPP
#define COMPUTE_THREAD_HPP

#include <memory>
#include <vector>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <json.hpp>

#include <teb_planner/teb_planner.hpp>
#include <teb_planner/robot_footprint_model.hpp>
#include <teb_planner/obstacles.hpp>
#include <teb_planner/teb_config.hpp>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class ComputeThread : public QThread
            {
                Q_OBJECT
            public:
                static std::shared_ptr<ComputeThread> getInstance();

                void setRobot(std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > robot);

                void setObstacles(std::shared_ptr< std::vector< std::shared_ptr< navigation::tebplanner::Obstacle > > > obstacles);

                void setConfig(std::shared_ptr< navigation::tebplanner::TebConfig > config);

                void setStart(std::shared_ptr< navigation::tebplanner::Pose > start);

                void setGoal(std::shared_ptr< navigation::tebplanner::Pose > goal);

                void stop();

                std::vector< navigation::tebplanner::TrajectoryPoint > trajectory() const;

            signals:
                void trajectoryUpdated();

            protected:
                virtual void run() override;

            private:
                explicit ComputeThread(QObject* parent = nullptr);

                static std::shared_ptr< ComputeThread > m_singleton;

                std::vector< geometry::Point > m_waypoints;
                std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > m_robot;
                std::shared_ptr< navigation::tebplanner::TebConfig > m_config;
                std::shared_ptr< std::vector< std::shared_ptr< navigation::tebplanner::Obstacle > > > m_obstacles;
                std::shared_ptr< navigation::tebplanner::Pose > m_start;
                std::shared_ptr< navigation::tebplanner::Pose > m_goal;
                std::unique_ptr< navigation::tebplanner::TebPlanner > m_planner;
            };
        }
    }
}

#endif // COMPUTE_THREAD_HPP
