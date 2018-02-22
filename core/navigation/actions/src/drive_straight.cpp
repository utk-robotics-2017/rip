#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                 const units::Distance& distance, double p, double i, double d)
                : Action(name)
                , m_use_time(false)
                , m_distance(distance)
                , m_drivetrain(drivetrain)
            {}

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                 const units::Time& time, const units::Velocity& speed)
                : Action(name)
                , m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_drivetrain(drivetrain)
            {}

            bool DriveStraight::isFinished()
            {
                std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
                units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(current - m_start_time).count() * units::ms;
                misc::Logger::getInstance()->debug("Diff time: {}", diff.to(units::s));
                return  diff >= m_time;
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                return;
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                //misc::Logger::getInstance()->debug("Start time: {}", m_start_time.to(units::s));
                motorcontrollers::MotorDynamics dynamics;
                dynamics.setSpeed(m_speed);
                m_drivetrain->drive(dynamics);
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
