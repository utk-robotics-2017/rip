#ifndef NAV_COMMAND_HPP
#define NAV_COMMAND_HPP

namespace rip
{
    namespace subsystem
    {
        class NavCommand
        {
        public:
            NavCommand(const Velocity& velocity, const Acceleration& acceleration);

            Velocity velocity() const;

            Acceleration acceleration() const;

        private:
            Velocity m_velocity;
            Acceleration m_acceleraton;
        };
    }
}


#endif // NAV_COMMAND_HPP
