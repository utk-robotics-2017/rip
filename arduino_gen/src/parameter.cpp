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
            : XmlElement(xml), m_id(id)
        {
            m_name = getAttribute("name")->Value();
            m_type = getAttribute("type")->Value();

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Parameter on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Parameter on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string Parameter::receive() const
        {
            return fmt::format(
                "\t{type} {name} = cmdMessenger.readBinArg<{type}>();\n"
                "\tif(!cmdMessenger.isArgOk()) {{\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::{id});\n"
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
