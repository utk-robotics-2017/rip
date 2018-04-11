#include "navigation_actions/turn_to_angle.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <navigation_actions/exceptions.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            TurnToAngle::TurnToAngle(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain, std::shared_ptr<NavX> navx, const nlohmann::json& config)
                : Action(name)
                , m_drivetrain(drivetrain)
                , m_navx(navx)
                , m_first(true)
            {
                m_turn_angle = config["turn_angle"];
                m_pid = std::unique_ptr<pid::PidController>(new pid::PidController(navx.get(), this, misc::constants::get<double>(misc::constants::kTurnKp), misc::constants::get<double>(misc::constants::kTurnKi),  misc::constants::get<double>(misc::constants::kTurnKd)));
                m_pid->setPercentTolerance(1);
                m_navx->setType(pid::PidInput::Type::kDisplacement);
                m_pid->setContinuous(true);
                m_pid->setInputRange(-180 * units::deg(), 180 * units::deg());
                m_pid->setOutputRange(-18 * units::in() / units::s(), 18 * units::in() / units::s());

                m_max_accel = misc::constants::get<units::Acceleration>(misc::constants::kMaxAcceleration);
            }

            bool TurnToAngle::isFinished()
            {
                return m_pid->onTarget();
            }

            void TurnToAngle::update(nlohmann::json& state)
            {
                units::Angle angle = m_navx->getYaw();
                misc::Logger::getInstance()->debug("Conn: {},Setpoint: {} deg,  TurnToAngle: {} deg", m_navx->isConnected(), m_setpoint.to(units::deg), angle.to(units::deg));
                m_pid->calculate();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            void TurnToAngle::setup(nlohmann::json& state)
            {
                m_start_angle = m_navx->getYaw();
                m_setpoint = m_start_angle + m_turn_angle;
                while(m_setpoint > 180 * units::deg())
                {
                    m_setpoint -= 360 * units::deg;
                }
                while(m_setpoint < -180 * units::deg())
                {
                    m_setpoint += 360 * units::deg;
                }
                misc::Logger::getInstance()->debug("Start: {} deg, Setpoint: {} deg", m_start_angle.to(units::deg), m_setpoint.to(units::deg));
                m_pid->setSetpoint(m_setpoint());
                m_pid->calculate();
            }

            void TurnToAngle::teardown(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Degrees turned: {} deg", (m_navx->getFusedHeading() - m_start_angle).to(units::deg));
                m_drivetrain->stop();
            }

            void TurnToAngle::set(double output)
            {
                if(m_first)
                {
                    motorcontrollers::MotorDynamics left;
                    motorcontrollers::MotorDynamics right;

                    left.setSpeed(output);
                    right.setSpeed(-output);

                    left.setAcceleration(m_max_accel);
                    right.setAcceleration(m_max_accel);

                    m_drivetrain->drive(left, right);
                    m_first = false;
                }
            }
        }
    }
}
