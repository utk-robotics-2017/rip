#ifndef COMPUTE_THREAD_HPP
#define COMPUTE_THREAD_HPP

#include <memory>
#include <vector>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QReadWriteLock>

#include <spline_planner/waypoint.hpp>
#include <spline_planner/path_planner.hpp>

#include <spline_planner/cubic_hermite_spline.hpp>

#include <misc/settings_base.hpp>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            class ComputeThread : public QThread
            {
                using Waypoint = navigation::Waypoint;
                using PathPlanner = navigation::PathPlanner<navigation::CubicHermiteSpline>;
                Q_OBJECT
            public:
                static std::shared_ptr<ComputeThread> getInstance();

                ~ComputeThread();

                void updateRobot(std::shared_ptr<misc::SettingsBase> settings);

                void setWaypoints(const std::vector<Waypoint>& waypoints);
                std::vector<Waypoint> waypoints();

                void setWidth(const Distance& width);
                Distance width();

                void setLength(const Distance& length);
                Distance length();

                void setMaxVelocity(const Velocity& max_v);
                Velocity maxVelocity();

                void setMaxAcceleration(const Acceleration& max_a);
                Acceleration maxAcceleration();

                void setMaxJerk(const Jerk& max_j);
                Jerk maxJerk();

                std::vector< std::array<geometry::Point, 3> > trajectory(const units::Time& dt);
                std::vector< std::array<geometry::Point, 3> > trajectory(const units::Time& dt, units::Time total_time, bool& done);

                std::tuple<Distance, Time> stats();

            signals:
                void newPlan();
                void newWaypoints();

            protected:
                void run() override;

            private:
                explicit ComputeThread(QObject* parent = nullptr);

                static std::shared_ptr<ComputeThread> m_singleton;
                std::vector<Waypoint> m_waypoints;
                std::unique_ptr<PathPlanner> m_planner;

                Distance m_width;
                Distance m_length;
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
