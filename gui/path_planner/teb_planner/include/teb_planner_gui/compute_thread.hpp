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

                void updateConfig(const nlohmann::json& config);

                void updateObstacles(const std::vector< std::shared_ptr< navigation::Obstacle > >& obstacles);

                void updateStart(const navigation::Pose& start);

                void updateGoal(const navigation::Pose& goal);

                void updateWaypoints(const std::vector< geometry::Point >& waypoints);

                std::vector< navigation::TrajectoryPoint > trajectory() const;

                void stop();

            protected:
                virtual void run() override;

            private:
                explicit ComputeThread(QObject* parent = nullptr);

                static std::shared_ptr< ComputeThread > m_singleton;

                QMutex m_mutex;
                QWaitCondition m_wait_condition;

                std::shared_ptr< navigation::TebConfig > m_config;
                std::vector< std::shared_ptr< navigation::Obstacle > > m_obstacles;
                std::vector< geometry::Point > m_waypoints;
                std::shared_ptr< navigation::RobotFootprintModel > m_robot;
                navigation::Pose m_start;
                navigation::Pose m_goal;
                std::unique_ptr< navigation::TebOptimalPlanner > m_planner;
            };
        }
    }
}

#endif // COMPUTE_THREAD_HPP
