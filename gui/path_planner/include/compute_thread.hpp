#ifndef COMPUTE_THREAD_HPP
#define COMPUTE_THREAD_HPP

#include <memory>
#include <vector>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QReadWriteLock>

#include <waypoint.hpp>
#include <path_planner.hpp>

#include <cubic_hermite_spline.hpp>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class ComputeThread : public QThread
            {
                using Waypoint = navigation::pathplanner::Waypoint;
                using PathPlanner = navigation::pathplanner::PathPlanner<navigation::pathplanner::CubicHermiteSpline>;
                Q_OBJECT
            public:
                static std::shared_ptr<ComputeThread> getInstance();

                void setWaypoints(const std::vector<Waypoint>& waypoints);
                std::vector<Waypoint> waypoints();

                void setWidth(const Distance& width);
                Distance width();

                void setMaxVelocity(const Velocity& max_v);
                Velocity maxVelocity();

                void setMaxAcceleration(const Acceleration& max_a);
                Acceleration maxAcceleration();

                void setMaxJerk(const Jerk& max_j);
                Jerk maxJerk();

            protected:
                void run() override;

            private:
                explicit ComputeThread(QObject* parent = nullptr);

                static std::shared_ptr<ComputeThread> m_singleton;
                std::vector<Waypoint> m_waypoints;
                std::unique_ptr<PathPlanner> m_planner;

                Distance m_width;
                Velocity m_max_velocity;
                Acceleration m_max_acceleration;
                Jerk m_max_jerk;

                QMutex m_mutex;
                QWaitCondition m_wait_condition;
                QReadWriteLock m_var_lock;
                QReadWriteLock m_planner_lock;

            };
        }
    }
}

#endif // COMPUTE_THREAD_HPP
