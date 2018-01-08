#include "nav_command.hpp"

namespace rip
{
    namespace subsystem
    {
        NavCommand::NavCommand(const Velocity& velocity, const Acceleration& acceleration)
            : m_velocity(velocity)
            , m_acceleraton(acceleration)
        {}

        Velocity NavCommand::velocity() const
        {
            return m_velocity;
        }

        Acceleration NavCommand::acceleration() const
        {
            return m_acceleraton;
        }

    }
}
