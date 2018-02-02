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

            void ComputeThread::setRobot(std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > robot)
            {
                m_robot = robot;
            }

            void ComputeThread::setObstacles(std::shared_ptr < std::vector< std::shared_ptr< navigation::tebplanner::Obstacle > > > obstacles)
            {
                m_obstacles = obstacles;
            }

            void ComputeThread::setConfig(std::shared_ptr<navigation::tebplanner::TebConfig> config)
            {
                m_config = config;
            }

            void ComputeThread::setStart(std::shared_ptr< navigation::tebplanner::Pose > start)
            {
                m_start = start;
            }

            void ComputeThread::setGoal(std::shared_ptr< navigation::tebplanner::Pose > goal)
            {
                m_goal = goal;
            }

            std::vector<navigation::tebplanner::TrajectoryPoint> ComputeThread::trajectory() const
            {
                return m_planner->getTrajectory();
            }

            void ComputeThread::run()
            {
                if(m_config && m_robot && m_obstacles && m_start && m_goal)
                {
                    m_planner.reset(new navigation::tebplanner::TebPlanner(m_config, m_obstacles, m_robot));
                    m_planner->plan(*m_start, *m_goal);
                    emit trajectoryUpdated();
                }
            }

            ComputeThread::ComputeThread(QObject* parent)
                : QThread(parent)
                , m_config(nullptr)
                , m_robot(nullptr)
                , m_obstacles(nullptr)
                , m_start(nullptr)
                , m_goal(nullptr)
                , m_planner(nullptr)
            {
            }
        }
    }
}
