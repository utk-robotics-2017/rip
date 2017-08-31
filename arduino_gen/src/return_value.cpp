#include "return_value.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace arduinogen
{
    ReturnValue::ReturnValue(tinyxml2::XMLElement* xml)
    {
        const char* name = xml->Attribute("name");
        if(!name)
        {
            throw AttributeException(fmt::format("Return Value name missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_name = name;

        const char* type = xml->Attribute("type");
        if(!type)
        {
            throw AttributeException(fmt::format("Return Value type missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_type = type;
    }

    std::string ReturnValue::declare() const
    {
        return fmt::format("\t{} {};\n", m_type, m_name);
    }

    std::string ReturnValue::send() const
    {
        return fmt::format("\tcmdMessenger.sendBinArg({});\n", m_name);
    }
}
