#include "navigation_actions/turn_to_angle.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <navigation_actions/exceptions.hpp>
#include <chrono>
#include <thread>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            TurnToAngle::TurnToAngle(const std::string& name,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                std::shared_ptr<NavX> navx, const units::Distance& radius)
                 : Action(name)
                 , m_drivetrain(drivetrain)
                 , m_speed(speed)
                 , m_desiredAngle(angle)
                 , m_navx(navx)
                 , m_c2wRadius(radius)
            {}

            bool TurnToAngle::isFinished()
            {
                units::Angle currentAngle = m_navx->getAngle();

                if(m_priorAngle() != currentAngle())
                {
                    misc::Logger::getInstance()->debug(fmt::format("degrees turned: {}"
                    , currentAngle.to(units::deg) - m_init.to(units::deg)));
                    misc::Logger::getInstance()->debug(fmt::format("current: {}"
                    , currentAngle.to(units::deg)));
                }
                m_priorAngle = currentAngle;
                misc::Logger::getInstance()->debug(fmt::format("degrees turned: {}"
                , std::abs(currentAngle.to(units::deg) - m_init.to(units::deg))));
                return std::abs(currentAngle.to(units::deg) - m_init.to(units::deg)) >= std::abs(m_desiredAngle.to(units::deg));
            }

            void TurnToAngle::update(nlohmann::json& state)
            {
                return;
            }

            void TurnToAngle::setup(nlohmann::json& state)
            {
                //initial angle
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                m_init = m_navx->getYaw();
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;
                misc::Logger::getInstance()->debug(fmt::format("in place turn intended angular velocity (rev/min): {}"
                , m_speed.to(units::rev / units::minute)));

                if(m_desiredAngle() < 0)
                {
                    throw OutofBoundsException("degrees should be positive. Sign of velocity determines turning direction");
                }
                misc::Logger::getInstance()->debug(fmt::format("wheel linear speed (in/s): {}"
                , (m_speed * m_c2wRadius / units::rad).to(units::in / units::s)));
                misc::Logger::getInstance()->debug(fmt::format("init: {}"
                , m_init.to(units::deg)));

                dynamicsLeft.setSpeed(m_speed * m_c2wRadius / units::rad);
                dynamicsRight.setSpeed(-1 * m_speed * m_c2wRadius / units::rad);
                m_drivetrain->drive(dynamicsLeft, dynamicsRight);
            }

            void TurnToAngle::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
