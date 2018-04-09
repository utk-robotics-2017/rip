#include "pid/filter.hpp"

namespace rip
{
    namespace pid
    {
        Filter::Filter(PidInput* input)
            : m_input(std::move(input))
        {

        }
    }
}
