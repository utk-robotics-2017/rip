#include "navigation_actions/drive_straight.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(std::shared_ptr<drivetrains::DriveTrain> drivetrain, const units::Distance& distance, double p, double i, double d)
                : m_use_time(false)
                , m_distance(distance)
                , m_drivetrain(drivetrain)
            {}

            DriveStraight::DriveStraight(std::shared_ptr<drivetrains::DriveTrain> drivetrain, const units::Time& time, const units::Velocity& speed)
                : m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_drivetrain(drivetrain)
            {}

            bool DriveStraight::isFinished()
            {
                return duration_cast< milliseconds >(
                           system_clock::now().time_since_epoch()) - m_start_time.to(units::ms) > m_time.to(units::ms);
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                return;
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                m_start_time = duration_cast< milliseconds >(
                                   system_clock::now().time_since_epoch());
                m_drivetrain->drive(m_speed);
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
