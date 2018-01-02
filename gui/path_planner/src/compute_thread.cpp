#include "compute_thread.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            std::shared_ptr<ComputeThread> ComputeThread::m_singleton = nullptr;

            std::shared_ptr<ComputeThread> ComputeThread::getInstance()
            {
                if (!m_singleton)
                {
                    m_singleton = std::shared_ptr<ComputeThread>(new ComputeThread(nullptr));
                }
                return m_singleton;
            }

            ComputeThread::~ComputeThread()
            {
                requestInterruption();
                m_wait_condition.wakeAll();
            }

            void ComputeThread::updateRobot(std::shared_ptr<misc::SettingsBase> settings)
            {
                QWriteLocker lock(&m_var_lock);
                m_width = settings->get<Distance>("width");
                m_max_velocity = settings->get<Velocity>("max_velocity");
                m_max_acceleration = settings->get<Acceleration>("max_acceleration");
                m_max_jerk = settings->get<Jerk>("max_jerk");
                m_wait_condition.wakeAll();
            }

            void ComputeThread::setWaypoints(const std::vector<Waypoint>& waypoints)
            {
                QWriteLocker lock(&m_var_lock);
                m_waypoints = waypoints;
                emit newWaypoints();
                QWriteLocker plock(&m_planner_lock);
                if(m_waypoints.size() >= 2)
                {
                    m_planner.reset(new PathPlanner(m_waypoints));
                    m_wait_condition.wakeAll();
                }
            }

            std::vector<navigation::Waypoint> ComputeThread::waypoints()
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

            void ComputeThread::setLength(const Distance& length)
            {
                QWriteLocker lock(&m_var_lock);
                m_length = length;
                m_wait_condition.wakeAll();
            }

            Distance ComputeThread::length()
            {
                QReadLocker lock(&m_var_lock);
                return m_length;
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

            std::vector< std::array<geometry::Point, 3> > ComputeThread::trajectory(const units::Time& dt)
            {
                QReadLocker lock(&m_planner_lock);
                if(m_planner)
                {
                    std::vector< std::array<geometry::Point, 3> > rv;

                    Time total_time = m_planner->totalTime();
                    Time t = 0;
                    while(t <= total_time)
                    {
                        rv.push_back({m_planner->leftPosition(t), m_planner->centerPosition(t), m_planner->rightPosition(t)});
                        t += dt;
                    }
                    // One final stop
                    rv.push_back({m_planner->leftPosition(total_time), m_planner->centerPosition(total_time), m_planner->rightPosition(total_time)});

                    return rv;
                }
                return std::vector< std::array<geometry::Point, 3> >();
            }

            std::vector< std::array<geometry::Point, 3> > ComputeThread::trajectory(const units::Time& dt, units::Time total_time, bool& done)
            {
                QReadLocker lock(&m_planner_lock);
                if(m_planner)
                {
                    std::vector< std::array<geometry::Point, 3> > rv;
                    if(total_time >= m_planner->totalTime())
                    {
                        done = true;
                    }
                    Time t = 0;
                    while(t <= total_time)
                    {
                        rv.push_back({m_planner->leftPosition(t), m_planner->centerPosition(t), m_planner->rightPosition(t)});
                        t += dt;
                    }
                    // One final stop
                    rv.push_back({m_planner->leftPosition(total_time), m_planner->centerPosition(total_time), m_planner->rightPosition(total_time)});

                    return rv;
                }
                return std::vector< std::array<geometry::Point, 3> >();
            }

            std::tuple<Distance, Time> ComputeThread::stats()
            {
                QReadLocker lock(&m_planner_lock);
                if(m_planner)
                {

                    return std::tuple<Distance, Time>(m_planner->totalDistance(), m_planner->totalTime());
                }
                return std::tuple<Distance, Time>(0, 0);
            }

            void ComputeThread::run()
            {
                m_mutex.lock();
                forever
                {
                    m_wait_condition.wait(&m_mutex);
                    if(isInterruptionRequested())
                    {
                        break;
                    }
                    m_planner_lock.lockForWrite();
                    if(m_planner)
                    {
                        m_var_lock.lockForRead();
                        m_planner->setRobotConfig(m_width, m_max_velocity, m_max_acceleration, m_max_jerk);
                        m_planner->calculate();
                        emit newPlan();
                        m_var_lock.unlock();
                    }
                    m_planner_lock.unlock();
                }
                m_mutex.unlock();
            }

            ComputeThread::ComputeThread(QObject* parent)
                : QThread(parent)
                , m_planner(nullptr)
            {
            }

        }
    }
}
