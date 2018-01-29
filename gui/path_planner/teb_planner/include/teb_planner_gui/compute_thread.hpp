#ifndef COMPUTE_THREAD_HPP
#define COMPUTE_THREAD_HPP

#include <memory>
#include <vector>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <json.hpp>

#include <teb_planner/trajectory_point.hpp>
#include <teb_planner/obstacle.hpp>
#include <teb_planner/optimal_planner.hpp>
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

                void setRobot(std::shared_ptr< navigation::RobotFootprintModel > robot);

                void setObstacles(std::shared_ptr< std::vector< std::shared_ptr< navigation::Obstacle > > > obstacles);

                void setConfig(std::shared_ptr< navigation::TebConfig > config);

                void setStart(std::shared_ptr< navigation::Pose > start);

                void setGoal(std::shared_ptr< navigation::Pose > goal);

                void stop();

                std::vector< navigation::TrajectoryPoint > trajectory() const;

            signals:
                void trajectoryUpdated();

            protected:
                virtual void run() override;

            private:
                explicit ComputeThread(QObject* parent = nullptr);

                static std::shared_ptr< ComputeThread > m_singleton;

                std::vector< geometry::Point > m_waypoints;
                std::shared_ptr< navigation::RobotFootprintModel > m_robot;
                std::shared_ptr< navigation::TebConfig > m_config;
                std::shared_ptr< std::vector< std::shared_ptr< navigation::Obstacle > > > m_obstacles;
                std::shared_ptr< navigation::Pose > m_start;
                std::shared_ptr< navigation::Pose > m_goal;
                std::unique_ptr< navigation::TebOptimalPlanner > m_planner;
            };
        }
    }
}

#endif // COMPUTE_THREAD_HPP
