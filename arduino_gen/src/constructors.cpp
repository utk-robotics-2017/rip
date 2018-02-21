#include "arduino_gen/constructors.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Constructors::Constructors(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            // Loop through the parameters for the constructor
            for (const tinyxml2::XMLElement* ele : getElements("constructor"))
            {
                m_constructors.emplace_back(ele);
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Constructors on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Constructors on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string Constructors::toString(const std::vector<std::shared_ptr<Appendage>>& appendages) const
        {
            std::string rv = "";

            for (Constructor constructor : m_constructors)
            {
                rv += constructor.toString(appendages) + "\n";
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
            }

            return rv;
        }
    }
}
