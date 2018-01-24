#include "includes.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "constructors.hpp"
#include "setup.hpp"
#include "loop.hpp"
#include "command.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        //Includes::Includes() = default;
        //Includes::~Includes() = default;

        Includes::Includes(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            for (const tinyxml2::XMLElement* includeElement : getElements("include"))
            {
                XmlElement include(includeElement);

                bool standard;
                try
                {
                    standard = include.getAttribute("standard")->BoolValue();
                }
                catch (AttributeException)
                {
                    standard = false;
                }

                if (!include.isAttributesEmpty())
                {
                    throw AttributeException(fmt::format("Extra attribute for Include on line number {}",
                                                         include.getLineNum()));
                }

                if (!include.isElementsEmpty())
                {
                    throw ElementException(fmt::format("Extra element for Include on line number {}",
                                                       include.getLineNum()));
                }

                if (standard)
                {
                    m_includes.emplace_back(fmt::format("<{0}>", include.getText()));
                }
                else
                {
                    m_includes.emplace_back(fmt::format("\"{0}\"", include.getText()));
                }
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Includes on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Includes on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::vector<std::string> Includes::GetIncludes()
        {
            return m_includes;
        }
    }
}
