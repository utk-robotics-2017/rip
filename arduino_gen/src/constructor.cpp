#include "arduino_gen/constructors.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "arduino_gen/argument.hpp"
#include "arduino_gen/appendage.hpp"
#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Constructor::Constructor(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            m_type = getAttribute("type")->Value();
            m_variable = getAttribute("variable")->Value();

            try
            {
                m_type_is_class = getAttribute("type-is-class")->BoolValue();
            }
            catch (AttributeException)
            {
                m_type_is_class = true;
            }

            // Loop through the parameters for the constructor
            for (const tinyxml2::XMLElement* ele : getElements("argument"))
            {
                m_arguments.emplace_back(ele);
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

        std::string Constructor::toString(const std::vector<std::shared_ptr<Appendage>>& appendages) const
        {
            std::string rv;

            rv += fmt::format("{} {} [{}] = {{\n", m_type, m_variable, appendages.size());

            for(std::shared_ptr<Appendage> appendage : appendages)
            {
                rv += "\t";
                
                if (m_type_is_class)
                {
                    rv += fmt::format("{}(", m_type);
                }

                for(const Argument& argument : m_arguments)
                {
                    if (appendage->has(argument.getName()))
                    {
						if (argument.getType() == "string")
						{
							rv += fmt::format("\"{}\"", argument.toString(appendage));
						}
                        else
                        {
                            rv += argument.toString(appendage);
                        }
                    }
                    else
                    {
                        std::string value = argument.getValue();
                        if (value.size() > 0)
                        {
                            rv += value;
                        }
                        else
                        {
                            // TODO(Anthony): Throw Exception
                        }
                    }

                    rv += ", ";
                }

                // Remove last comma and add the end of the single constructor
                if (m_arguments.size() > 0)
                {
                    rv = rv.substr(0, rv.size() - 2);
                }

                if (m_type_is_class)
                {
                    rv += ")";
                }

                rv += ",\n";
            }

            // Remove the last comma and new line and add the end of the constructor list
            if (appendages.size() > 0)
            {
                rv = rv.substr(0, rv.size() - 2) + "\n";
            }
            rv += "};\n";

            return rv;
        }
    } // arduinogen
}
