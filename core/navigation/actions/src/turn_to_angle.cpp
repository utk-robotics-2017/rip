#include "navigation_actions/turn_to_angle.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            TurnToAngle::TurnToAngle(const std::string& name,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                std::shared_ptr<NavX> navx, units::Distance radius)
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

                if(m_desiredAngle >= 0)
                {
                    return currentAngle >= m_desiredAngle;
                }
                else
                {
                    return currentAngle <= m_desiredAngle;
                }
            }

            void TurnToAngle::update(nlohmann::json& state)
            {
                return;
            }

            void TurnToAngle::setup(nlohmann::json& state)
            {
                //initial angle
                misc::Logger::getInstance()->debug("setting yaw offset");
                m_navx->zeroYaw();
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;
                misc::Logger::getInstance()->debug(fmt::format("in place turn intended angular velocity (rev/min): {}"
                , m_speed.to(units::rev / units::minute)));
                if(m_speed() < 0)
                {
                    m_speed *= -1;
                }
                if(m_desiredAngle < 0)
                {
                    m_speed *= -1;
                }
                misc::Logger::getInstance()->debug(fmt::format("wheel linear speed (in/s): {}"
                , (m_speed * m_c2wRadius / units::rad).to(units::in / units::s)));
                
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
