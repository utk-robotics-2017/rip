#include "navigation_actions/turn_by_angle.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <navigation_actions/exceptions.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {

            void TurnByAngle::setup(nlohmann::json& state)
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
                TimeoutAction::setup(state);
            }

        }
    }
}
