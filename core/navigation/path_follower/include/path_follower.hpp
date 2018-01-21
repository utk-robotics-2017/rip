#ifndef PATH_FOLLOWER_HPP
#define PATH_FOLLOWER_HPP

#include <memory>

#include <units.hpp>
#include <point.hpp>
#include <path_planner.hpp>
#include <drivetrain.hpp>

namespace rip
{
    namespace navigation
    {
        /**
         * A simple path follower that directs the motion profile to the drive train
         *
         * A more complicated Adaptive Pure Pursuit Controller will be integrated if necessary
         */
        class PathFollower
        {
        public:
            PathFollower(const pathplanner::PathPlanner& planner, std::shared_ptr<drivetrain::DriveTrain> drive_train)
                : m_planner(planner)
                , m_drivetrain(drivetrain)
            {}

            void update(const units::Time& t)
            {
                units::Time dt = t - m_previous.time();

                geometry::Point left_p = m_planner.leftPosition(t);

                units::Velocity left_v = left_p.distance(m_previous_left_position) / dt;
                units::Acceleration left_a = (left_v - m_previous_left_velocity) / dt;
                m_previous_left_position = left_p;
                m_previous_left_velocity = left_v;
                m_previous_left_acceleration = left_a;


                units::Velocity right_v = right_p.distance(m_previous_right_position) / dt;
                units::Acceleration right_a = (right_v - m_previous_right_velocity) / dt;
                m_previous_right_position = right_p;
                m_previous_right_velocity = right_v;
                m_previous_right_acceleration = right_a;

                NavCommand left(left_v, left_a);
                NavCommand right(right_v, right_a);

                m_drive_train->drive(left, right);
            }

            bool finished(const units::Time& t)
            {
                return t >= m_planner.totalTime();
            }

        private:
            pathplanner::PathPlanner m_planner;
            std::shared_ptr<drivetrain::DriveTrain> m_drive_train;

            geometry::Point m_previous_left_position;
            units::Velocity m_previous_left_velocity;
            units::Acceleration m_previous_left_acceleration;

            geometry::Point m_previous_right_position;
            units::Velocity m_previous_right_velocity;
            units::Acceleration m_previous_right_acceleration;
        };

    }
}



#endif // PATH_FOLLOWER_HPP
