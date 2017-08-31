#include "argument.hpp"

#include <tinyxml2.h>
#include <fmt/format.h>

#include "appendage.hpp"
#include "exceptions.hpp"

namespace arduinogen
{
    Argument::Argument(tinyxml2::XMLElement* xml)
    {
        const char* name = xml->Attribute("name");
        if(!name)
        {
            throw AttributeException(fmt::format("Constructor argument name missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_name = name;

        const char* type = xml->Attribute("type");
        if(!type)
        {
            throw AttributeException(fmt::format("Constructor argument type missing on line number {}",
                                                 xml->GetLineNum()));
        }
        m_type = type;

        if(m_type != "float" &&
           m_type != "int" &&
           m_type != "bool" &&
           m_type != "string")
        {
            throw AttributeException(fmt::format("Constructor argument unknown type on line number {}",
                                                 xml->GetLineNum()));
        }
    }

    std::string Argument::toString(std::shared_ptr<Appendage> appendage) const
    {
        if(!appendage->isType(m_name, m_type))
        {
            // TODO(Andrew): throw exception
        }
        return fmt::format("{}, ", appendage->getString(m_name));
    }
} // arduinogen
