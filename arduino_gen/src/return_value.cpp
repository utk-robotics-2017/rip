#include "return_value.hpp"

#include <map>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        ReturnValue::ReturnValue(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            m_name = getAttribute("name")->Value();
            m_type = getAttribute("type")->Value();

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for ReturnValue on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for ReturnValue on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string ReturnValue::declare() const
        {
            return fmt::format("\t{} {};\n", m_type, m_name);
        }

        std::string ReturnValue::send() const
        {
            return fmt::format("\tcmdMessenger.sendBinArg({});\n", m_name);
        }

        std::string ReturnValue::getName() const
        {
            return m_name;
        }

        void ReturnValue::setName(const std::string& name)
        {
            m_name = name;
        }

        std::string ReturnValue::getType() const
        {
            return m_type;
        }

        void ReturnValue::setType(const std::string& type)
        {
            m_type = type;
        }
    }
}
