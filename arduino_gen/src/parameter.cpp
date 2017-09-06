#include "parameter.hpp"

#include <map>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Parameter::Parameter(const tinyxml2::XMLElement* xml, std::string id)
            : m_id(id)
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

        std::string Parameter::receive() const
        {
            return fmt::format(
                "\t{type} {name} = cmdMessenger.readBinArg<{type}>();\n"
                "\tif(!cmdMessenger.isArgOk()) {{\n"
                "\t\tcmdMessenger.sendBinCmd(kError, {id});\n"
                "\t\treturn;\n"
                "\t}}\n",
                fmt::arg("name", m_name),
                fmt::arg("type", m_type),
                fmt::arg("id", m_id));
        }

        std::string Parameter::getName() const
        {
            return m_name;
        }

        void Parameter::setName(const std::string& name)
        {
            m_name = name;
        }

        std::string Parameter::getType() const
        {
            return m_type;
        }

        void Parameter::setType(const std::string& type)
        {
            m_type = type;
        }

        std::string Parameter::getId() const
        {
            return m_id;
        }

        void Parameter::setId(const std::string& id)
        {
            m_id = id;
        }
    }
}
