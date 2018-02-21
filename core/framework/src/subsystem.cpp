#include "framework/subsystem.hpp"

namespace rip
{
    namespace framework
    {

        Subsystem::Subsystem(const std::string& name)
            : m_name(name)
        {}

        Subsystem::~Subsystem()
        {
        }

        void Subsystem::setName(const std::string& name)
        {
            m_name = name;
        }

        std::string Subsystem::name() const
        {
            return m_name;
        }

    }
}
