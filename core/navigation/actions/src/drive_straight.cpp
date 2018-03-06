#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain, std::shared_ptr<navx::NavX> navx,
                                         const units::Distance& distance, const units::Velocity& speed, double p, double i, double d)
                : Action(name)
                , m_use_time(false)
                , m_distance(distance)
                , m_speed(speed)
                , m_drivetrain(drivetrain)
                , m_pid(new pid::PidController(m_navx.get(), this, p, i , d))
            {}

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain, std::shared_ptr<navx::NavX> navx,
                                         const units::Time& time, const units::Velocity& speed, double p, double i, double d)
                : Action(name)
                , m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_drivetrain(drivetrain)
                , m_pid(new pid::PidController(m_navx.get(), this, p, i , d))
            {}

            bool DriveStraight::isFinished()
            {
                if(m_use_time)
                {
                    std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
                    units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(current - m_start_time).count() * units::ms;
                    misc::Logger::getInstance()->debug("Diff time: {}", diff.to(units::s));
                    return  diff >= m_time;
                }
                else
                {
                    // todo: get distance check distance
                    return true;
                }
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                // todo: change velocities based on heading
                m_pid->calculate();
            }

            void DriveStraight::set(double value)
            {
                motorcontrollers::MotorDynamics left;
                left.setSpeed(m_speed - units::Velocity(value));
                motorcontrollers::MotorDynamics right;
                right.setSpeed(m_speed + units::Velocity(value));
                m_drivetrain->drive(left, right);
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
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
