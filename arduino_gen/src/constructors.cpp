#include "constructors.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "argument.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Constructors::Constructors(tinyxml2::XMLElement* xml)
            : m_exists(true)
        {
            if (!xml)
            {
                m_exists = false;
                return;
            }

            const char* type = xml->Attribute("type");
            if (!type)
            {
                throw AttributeException(fmt::format("Constructor type missing on line number {}",
                                                     xml->GetLineNum()));
            }
            m_type = type;

            const char* variable = xml->Attribute("variable");
            if (!variable)
            {
                throw AttributeException(fmt::format("Constructor variable missing on line number {}",
                                                     xml->GetLineNum()));
            }
            m_variable = variable;

            // Loop through the parameters for the constructor
            for (tinyxml2::XMLElement* argument = xml->FirstChildElement("argument");
                    argument != nullptr; argument = argument->NextSiblingElement("argument"))
            {
                m_arguments.emplace_back(argument);
            }
        }

        std::string Constructors::toString(std::vector< std::shared_ptr<Appendage> >& appendages) const
        {
            if (!m_exists)
            {
                return "";
            }

            std::string rv;

            rv += fmt::format("{} {} = {{\n", m_type, m_variable);

            for (std::shared_ptr<Appendage> appendage : appendages)
            {
                rv += fmt::format("\t{}(", m_type);

                for (const Argument& argument : m_arguments)
                {
                    rv += argument.toString(appendage);
                }

                // Remove last comma and add the end of the single constructor
                rv = rv.substr(0, rv.size() - 2) + fmt::format("),\n");
            }

            // Remove the last comma and new line and add the end of the constructor list
            rv = rv.substr(0, rv.size() - 2) + "\n};\n";

            return rv;
        }
    } // arduinogen
}