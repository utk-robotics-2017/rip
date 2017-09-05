#include "command.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "argument.hpp"
#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Command::Command(tinyxml2::XMLElement* xml)
        {
            const char* id = xml->Attribute("id");
            if (!id)
            {
                throw AttributeException(fmt::format("Command id missing on line number {}",
                                                     xml->GetLineNum()));
            }
            m_id = id;

            const char* name = xml->Attribute("name");
            if (!name)
            {
                throw AttributeException(fmt::format("Command name missing on line number {}",
                                                     xml->GetLineNum()));
            }
            m_name = name;

            // Index num defaults to true
            bool index_num = xml->BoolAttribute("index-num", true);
            m_index_num = index_num;
            // Add the code for the parameters received from RIP
            for (tinyxml2::XMLElement* parameter = xml->FirstChildElement("parameter"); parameter != nullptr;
                    parameter = parameter->NextSiblingElement("parameter"))
            {
                m_parameters.emplace_back(parameter, id);
            }

            // Add declarations for the value to send back to RIP
            for (tinyxml2::XMLElement* return_value = xml->FirstChildElement("return-value");
                    return_value != nullptr; return_value = return_value->NextSiblingElement("return-value"))
            {
                m_return_values.emplace_back(return_value);
            }


            tinyxml2::XMLElement* code = xml->FirstChildElement("code");
            if (!code)
            {
                // ??? Is this supposed to be an AttributeException, or something else we need to define?
                // TODO(Andrew): Define another exception
                throw AttributeException(fmt::format("Command {} missing code on line number {}", id,
                                                     xml->GetLineNum()));
            }
            m_code = code->GetText();
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
            return m_code;
        }

        std::string Command::callback(int num_appendages) const
        {
            std::string rv = fmt::format("void {} {\n", m_name);
            if (m_index_num)
            {
                rv += "\tint indexNum = cmdMessenger.readBinArg<int>();\n";
                rv += fmt::format("\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > {}) {\n",
                                  num_appendages);
                rv += fmt::format("\t\tcmdMessenger.sendBinCmd(kError, {});\n", m_id);
                rv += "\t}\n";
            }
            for (const Parameter& parameter : m_parameters)
            {
                rv += parameter.receive();
            }

            for (const ReturnValue& return_value : m_return_values)
            {
                rv += return_value.declare();
            }

            rv += m_code;

            rv += fmt::format("\tcmdMessenger.sendBindCmd(kAcknowledge, {});\n", m_id);

            if (m_return_values.size())
            {
                rv += fmt::format("\tcmdMessenger.sendCmdStart({}Result);\n", m_id);

                for (const ReturnValue& return_value : m_return_values)
                {
                    rv += return_value.send();
                }

                rv += "\tcmdMeessenger.sendCmdEnd();\n";
            }

            return rv;
        }
    }
}