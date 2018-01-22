#include <teb_planner_gui/compute_thread.hpp>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            std::shared_ptr<ComputeThread> ComputeThread::m_singleton = nullptr;

            std::shared_ptr<ComputeThread> ComputeThread::getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::shared_ptr<ComputeThread>(new ComputeThread);
                }

                return m_singleton;
            }

            void ComputeThread::updateConfig(const nlohmann::json& config)
            {
                *m_config = config;
                m_wait_condition.wakeOne();
            }

            void ComputeThread::updateObstacles(const std::vector< std::shared_ptr< navigation::Obstacle > >& obstacles)
            {
                m_obstacles = obstacles;
                m_wait_condition.wakeOne();
            }

            void ComputeThread::updateStart(const navigation::Pose& start)
            {
                m_start = start;
                m_wait_condition.wakeOne();
            }

            void ComputeThread::updateGoal(const navigation::Pose& goal)
            {
                m_goal = goal;
                m_wait_condition.wakeOne();
            }

            void ComputeThread::updateWaypoints(const std::vector<geometry::Point>& waypoints)
            {
                m_waypoints = waypoints;
                m_wait_condition.wakeOne();
            }

            std::vector<navigation::TrajectoryPoint> ComputeThread::trajectory() const
            {
                if(m_planner)
                {
                    return m_planner->getTrajectory();
                }
                return std::vector<navigation::TrajectoryPoint>();
            }

            void ComputeThread::stop()
            {
                requestInterruption();
                m_wait_condition.wakeAll();
            }

            void ComputeThread::run()
            {
                /*
                forever
                {
                    m_wait_condition.wait(&m_mutex);
                    if(interruptionRequested())
                    {
                        break;
                    }
                    if(m_config && m_obstacles.size() && m_robot)
                    {
                        m_planner = std::unique_ptr<navigation::TebOptimalPlanner>(new navigation::TebOptimalPlanner(m_config, m_obstacles, m_robot, m_waypoints));
                        m_planner->plan(m_start, m_goal);
                    }
                    if(interruptionRequested())
                    {
                        break;
                    }
                }
                */
            }

            ComputeThread::ComputeThread(QObject* parent)
                : QThread(parent)
                , m_config(nullptr)
                , m_planner(nullptr)
                , m_robot(nullptr)
            {
            }
        }
    }
}
