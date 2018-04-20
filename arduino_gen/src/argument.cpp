#include "arduino_gen/argument.hpp"

#include <tinyxml2.h>
#include <fmt/format.h>

#include "arduino_gen/appendage.hpp"
#include "arduino_gen/exceptions.hpp"

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

			m_prefix = "";
			m_suffix = "";
			if (m_type == "string_literal")
			{
				try
				{
					m_prefix = getAttribute("prefix")->Value();
				}
				catch (AttributeException)
				{ }

				try
				{
					m_suffix = getAttribute("suffix")->Value();
				}
				catch (AttributeException)
				{ }
			}

            if(m_type != "float" &&
               m_type != "int" &&
               m_type != "bool" &&
               m_type != "string" &&
               m_type != "long" &&
               m_type != "string_literal")
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
				if (!appendage->isType(m_name, m_type == "string_literal" ? "string" : m_type))
				{
					// TODO(Andrew): throw exception
				}
				return m_prefix + appendage->getString(m_name) + m_suffix;
            }
            else
            {
                if (m_value.size() != 0)
                {
                    return m_value;
                }
                else
                {
                    throw AppendageDataException("Appendage doesn't have value for \"{}\" and argument doesn't have a default value.", m_name);
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
