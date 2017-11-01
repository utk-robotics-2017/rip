#include "setup_parameter.hpp"

#include <map>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        SetupParameter::SetupParameter(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            m_name = getAttribute("name")->Value();
            m_replacer = getAttribute("replacer")->Value();

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for SetupParameter on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for SetupParameter on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string SetupParameter::getName() const
        {
            return m_name;
        }

        std::string SetupParameter::getReplacer() const
        {
            return m_replacer;
        }
    }
}
