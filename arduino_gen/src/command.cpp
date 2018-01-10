#include "command.hpp"

#include <sstream>
#include <regex>
#include <list>
#include <map>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "code.hpp"
#include "parameter.hpp"
#include "return_value.hpp"
#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Command::Command(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            // Get the id attribute
            m_id = getAttribute("id")->Value();

            // Get the name attribute
            m_name = getAttribute("name")->Value();

            // Get the index-num attribute
            try
            {
                m_index_num = getAttribute("index-num")->BoolValue();
            }
            catch (const AttributeException& e)
            {
                // The index-num attribute is not in the XmlElement, default to true
                m_index_num = true;
            }

            // Gets all of the parameter child elements, construct Parameters and store in the vector
            for (const tinyxml2::XMLElement* ele : getElements("parameter"))
            {
                m_parameters.emplace_back(ele, m_id);
            }

            // Gets all of the return-value child elements, construct ReturnValue and store in the vector
            for (const tinyxml2::XMLElement* ele : getElements("return-value"))
            {
                m_return_values.emplace_back(ele);
            }

            // Ensure there is only one code element, then process the code text
            std::vector<const tinyxml2::XMLElement*> code_elements = getElements("code");
            if (code_elements.size() == 1)
            {
                m_code = std::unique_ptr<Code>(new Code(code_elements[0]));
            }
            else
            {
                throw ElementException("Command requires 1 code element");
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Command on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Command on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string Command::getId() const
        {
            return m_id;
        }

        std::string Command::getResultId() const
        {
            return m_return_values.size() ? m_id + "Result" : "";
        }

        std::string Command::getName() const
        {
            return m_name;
        }

        bool Command::getIndexNum() const
        {
            return m_index_num;
        }

        std::string Command::getCode() const
        {
            return m_code->getCode();
        }

        std::string Command::callback(int num_appendages) const
        {
            std::string rv = fmt::format("void {}() {{\n", m_name);
            if(m_index_num)
            {
                rv += fmt::format(
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > {num_appendages}) {{\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, {m_id});\n"
                    "\t\treturn;\n"
                    "\t}}\n",
                    fmt::arg("num_appendages", num_appendages),
                    fmt::arg("m_id", m_id));
            }
            for(const Parameter& parameter : m_parameters)
            {
                rv += parameter.receive();
            }

            for(const ReturnValue& return_value : m_return_values)
            {
                rv += return_value.declare();
            }

            rv += m_code->getCode();

            rv += fmt::format("\tcmdMessenger.sendBindCmd(kAcknowledge, {});\n", m_id);

            if(m_return_values.size())
            {
                rv += fmt::format("\tcmdMessenger.sendCmdStart({}Result);\n", m_id);

                for(const ReturnValue& return_value : m_return_values)
                {
                    rv += return_value.send();
                }

                rv += "\tcmdMeessenger.sendCmdEnd();\n";
            }

            rv += "}\n";

            return rv;
        }
    }
}
