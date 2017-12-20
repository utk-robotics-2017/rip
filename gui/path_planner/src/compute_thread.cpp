#include "compute_thread.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            std::shared_ptr<ComputeThread> m_singleton = nullptr;

            std::shared_ptr<ComputeThread> ComputeThread::getInstance()
            {
                if (!m_singleton)
                {
                    m_singleton = std::shared_ptr<ComputeThread>(new ComputeThread(nullptr));
                }
                return m_singleton;
            }

            void ComputeThread::setWaypoints(const std::vector<Waypoint>& waypoints)
            {
                QWriteLocker lock(&m_var_lock);
                m_waypoints = waypoints;
                m_planner.reset(new PathPlanner(m_waypoints));
                m_wait_condition.wakeAll();
            }

            std::vector<navigation::pathplanner::Waypoint> ComputeThread::waypoints()
            {
                QReadLocker lock(&m_var_lock);
                return m_waypoints;
            }

            void ComputeThread::setWidth(const Distance& width)
            {
                QWriteLocker lock(&m_var_lock);
                m_width = width;
                m_wait_condition.wakeAll();
            }

            Distance ComputeThread::width()
            {
                QReadLocker lock(&m_var_lock);
                return m_width;
            }

            void ComputeThread::setMaxVelocity(const Velocity& max_v)
            {
                QWriteLocker lock(&m_var_lock);
                m_max_velocity = max_v;
                m_wait_condition.wakeAll();
            }

            Velocity ComputeThread::maxVelocity()
            {
                QReadLocker lock(&m_var_lock);
                return m_max_velocity;
            }

            void ComputeThread::setMaxAcceleration(const Acceleration& max_a)
            {
                QWriteLocker lock(&m_var_lock);
                m_max_acceleration = max_a;
                m_wait_condition.wakeAll();
            }

            Acceleration ComputeThread::maxAcceleration()
            {
                QReadLocker lock(&m_var_lock);
                return m_max_acceleration;
            }

            void ComputeThread::setMaxJerk(const Jerk& max_j)
            {
                QWriteLocker lock(&m_var_lock);
                m_max_jerk = max_j;
                m_wait_condition.wakeAll();
            }

            Jerk ComputeThread::maxJerk()
            {
                QReadLocker lock(&m_var_lock);
                return m_max_jerk;
            }

            void ComputeThread::run()
            {
                m_mutex.lock();
                forever
                {
                    m_wait_condition.wait(&m_mutex);
                    m_planner_lock.lockForWrite();
                    if(m_planner)
                    {
                        m_var_lock.lockForRead();
                        m_planner->setRobotConfig(m_width, m_max_velocity, m_max_acceleration, m_max_jerk);
                        m_planner->calculate();
                        m_var_lock.unlock();
                    }
                    m_planner_lock.unlock();
                }
            }

            ComputeThread::ComputeThread(QObject* parent)
                : QThread(parent)
                , m_planner(nullptr)
            {
            }

        }
    }
}
