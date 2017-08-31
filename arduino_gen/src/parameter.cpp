#include "parameter.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace arduinogen
{
    Parameter::Parameter(tinyxml2::XMLElement* xml, std::string id)
        : m_id(id)
    {
        const char* name = xml->Attribute("name");
        if(!name)
        {
            throw AttributeException(fmt::format("Parameter name missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_name = name;

        const char* type = xml->Attribute("type");
        if(!type)
        {
            throw AttributeException(fmt::format("Parameter type missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_type = type;
    }

    std::string Parameter::receive() const
    {
        std::string rv = fmt::format("\t{1} {0} = cmdMessenger.readBinArg<{1}>()", m_name, m_type);
        rv += "\tif(!cmdMessenger.isArgOk()){\n";
        rv += fmt::format("\t\tcmdMessenger.sendBinCmd(kError, {});\n", m_id);
        rv += "\t\treturn;\n";
        rv += "\t}";
        return rv;
    }
}
