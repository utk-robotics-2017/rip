#include "pid/pid_input.hpp"

namespace rip
{
    namespace pid
    {
        void PidInput::setType(PidInput::Type type)
        {
            m_type = type;
        }

        PidInput::Type PidInput::type()
        {
            return m_type;
        }
    }
}
