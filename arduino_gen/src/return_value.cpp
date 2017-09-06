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
        {
            std::map<std::string, const tinyxml2::XMLAttribute*> attributes;

            // Loop through the attributes in the element and add them to the map
            // auto -> const tinyxml2::XMLAttribute*
            for (auto attr = xml->FirstAttribute(); attr != nullptr; attr = attr->Next())
            {
                attributes.emplace(attr->Name(), attr);
            }

            // Get the value for the name attribute, then erase it from the map
            try
            {
                m_name = attributes.at("name")->Value();
                attributes.erase("name");
            }
            catch (const std::out_of_range& e)
            {
                throw AttributeException(fmt::format("Parameter name missing on line number {}",
                                                     xml->GetLineNum()));
            }

            // Get the value for the type attribute, then erase it from the map
            try
            {
                m_type = attributes.at("type")->Value();
                attributes.erase("type");
            }
            catch (const std::out_of_range& e)
            {
                throw AttributeException(fmt::format("Parameter type missing on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra attributes in the map, throw an exception
            if (!attributes.empty())
            {
                throw AttributeException(fmt::format("Extra attribute for Parameter on line number {}",
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
