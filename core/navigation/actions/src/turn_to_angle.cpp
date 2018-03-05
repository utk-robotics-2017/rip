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
            TurnToAngle::TurnToAngle(const std::string& name,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                std::shared_ptr<NavX> navx, const units::Distance& radius)
                 : Action(name)
                 , m_drivetrain(drivetrain)
                 , m_speed(speed)
                 , m_desired_angle(angle)
                 , m_navx(navx)
                 , m_c2w_radius(radius)
            {}

            bool TurnToAngle::isFinished()
            {
                units::Angle current_angle = m_navx->getAngle();
                m_prior_angle = current_angle;
                misc::Logger::getInstance()->debug(fmt::format("current angle: {}"
                , current_angle.to(units::deg)));
                return std::abs(current_angle.to(units::deg) - m_init.to(units::deg)) >= std::abs(m_desired_angle.to(units::deg));
            }

            void TurnToAngle::update(nlohmann::json& state)
            {
                return;
            }

            void TurnToAngle::setup(nlohmann::json& state)
            {
                //initial angle

                m_init = m_navx->getAngle();
                do
                {
                    m_init = m_navx->getAngle();
                } while(m_init.to(units::deg) == 0);
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;
                misc::Logger::getInstance()->debug(fmt::format("in place turn intended angular velocity (rev/min): {}"
                , m_speed.to(units::rev / units::minute)));

                if(m_desired_angle() <= 0)
                {
                    throw OutofBoundsException("degrees should be positive. Sign of velocity determines turning direction");
                }
                if(m_c2w_radius <= 0)
                {
                    throw OutofBoundsException("wheel radius should be positive");
                }
                misc::Logger::getInstance()->debug(fmt::format("wheel linear speed (in/s): {}"
                , (m_speed * m_c2w_radius / units::rad).to(units::in / units::s)));
                misc::Logger::getInstance()->debug(fmt::format("init: {}"
                , m_init.to(units::deg)));

                dynamicsLeft.setSpeed(m_speed * m_c2w_radius / units::rad);
                dynamicsRight.setSpeed(-1 * m_speed * m_c2w_radius / units::rad);
                m_drivetrain->drive(dynamicsLeft, dynamicsRight);
            }

            void TurnToAngle::teardown(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug(fmt::format("degrees turned: {}"
                , (m_prior_angle - m_init).to(units::deg)));
                m_drivetrain->stop();
            }
        }
    }
}
