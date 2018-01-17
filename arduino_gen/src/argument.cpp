#include "argument.hpp"

#include <tinyxml2.h>
#include <fmt/format.h>

#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Argument::Argument() = default;
        Argument::~Argument() = default;

        Argument::Argument(const tinyxml2::XMLElement* xml) : XmlElement(xml)
        {
            m_name = getAttribute("name")->Value();
            m_type = getAttribute("type")->Value();

            try
            {
                m_value = getAttribute("value")->Value();
            }
            catch (AttributeException)
            {
                m_value = "";
            }

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
            if (appendage->has(m_name))
            {
                if(!appendage->isType(m_name, m_type))
                {
                    // TODO(Andrew): throw exception
                }
                return appendage->getString(m_name);
            }
            else
            {
                if (m_value.size() != 0)
                {
                    return m_value;
                }
                else
                {
                    // TODO(Anthony): throw exception
                }
            }
        }

        const std::string& Argument::getName() const
        {
            return m_name;
        }

        const std::string& Argument::getType() const
        {
            return m_type;
        }

        const std::string& Argument::getValue() const
        {
            return m_value;
        }
    } // arduinogen
}
