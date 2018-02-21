#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>

#include <chrono>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain, const units::Distance& distance, double p, double i, double d)
                : Action(name)
                , m_use_time(false)
                , m_distance(distance)
                , m_drivetrain(drivetrain)
            {}

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain, const units::Time& time, const units::Velocity& speed)
                : Action(name)
                , m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_drivetrain(drivetrain)
            {}

            bool DriveStraight::isFinished()
            {
                return std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() * units::ms - m_start_time > m_time;
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                return;
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::duration_cast< std::chrono::milliseconds >(
                                   std::chrono::system_clock::now().time_since_epoch()).count() * units::ms;
                motorcontrollers::MotorDynamics dynamics;
                dynamics.setVelocity(m_speed);
                m_drivetrain->drive(dynamics);
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
