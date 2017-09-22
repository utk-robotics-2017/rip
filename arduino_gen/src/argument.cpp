#include "argument.hpp"

#include <tinyxml2.h>
#include <fmt/format.h>

#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Argument::Argument(tinyxml2::XMLElement* xml) : XmlElement(xml)
        {
            m_name = getAttribute("name")->Value();
            m_type = getAttribute("type")->Value();

            if(m_type != "float" &&
               m_type != "int" &&
               m_type != "bool" &&
               m_type != "string")
            {
                throw AttributeException(fmt::format("Constructor argument unknown type on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Argument on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Argument on line number {}",
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
}
