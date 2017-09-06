#include "command.hpp"

#include <sstream>
#include <regex>
#include <list>
#include <map>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "argument.hpp"
#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Command::Command(const tinyxml2::XMLElement* xml)
        {
            std::map<std::string, const tinyxml2::XMLAttribute*> attributes;
            std::multimap<std::string, const tinyxml2::XMLElement*> elements;

            // Loop through the attributes in the element and add them to the map
            // auto -> const tinyxml2::XMLAttribute*
            for (auto attr = xml->FirstAttribute(); attr != nullptr; attr = attr->Next())
            {
                attributes.emplace(attr->Name(), attr);
            }

            // Get the value for the id attribute, then erase it from the map
            try
            {
                m_id = attributes.at("id")->Value();
                attributes.erase("id");
            }
            catch (const std::out_of_range& e)
            {
                throw AttributeException(fmt::format("Command id missing on line number {}",
                                                     xml->GetLineNum()));
            }

            // Get the value for the name attribute, then erase it from the map
            try
            {
                m_name = attributes.at("name")->Value();
                attributes.erase("name");
            }
            catch (const std::out_of_range& e)
            {
                throw AttributeException(fmt::format("Command name missing on line number {}",
                                                     xml->GetLineNum()));
            }

            // Index num defaults to true
            // Get the value for the index-num attribute, then erase it from the map
            try
            {
                m_index_num = attributes.at("index-num")->BoolValue();
                attributes.erase("index-num");
            }
            catch (const std::out_of_range& e)
            {
                m_index_num = true;
            }

            // If there are any extra attributes in the map, throw an exception
            if (!attributes.empty())
            {
                throw AttributeException(fmt::format("Extra attribute for Command on line number {}",
                                                     xml->GetLineNum()));
            }

            // Loop through the child elements and add them to the map
            // auto -> const tinyxml2::XMLElement*
            for (auto ele = xml->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
            {
                elements.emplace(ele->Name(), ele);
            }

            // Loop through the parameter elements, and remove them after being processed
            // auto -> std::pair<std::multimap<std::string, const tinyxml2::XMLElement*>::iterator,
            //                   std::multimap<std::string, const tinyxml2::XMLElement*>::iterator>
            auto parameter_range = elements.equal_range("parameter");
            for (auto it = parameter_range.first; it != parameter_range.second; it = elements.erase(it))
            {
                m_parameters.emplace_back(it->second, m_id);

            }

            // Loop through the return-value elements, and remove them after being processed
            // auto -> std::pair<std::multimap<std::string, const tinyxml2::XMLElement*>::iterator,
            //                   std::multimap<std::string, const tinyxml2::XMLElement*>::iterator>
            auto return_value_range = elements.equal_range("return-value");
            for (auto it = return_value_range.first; it != return_value_range.second; it = elements.erase(it))
            {
                m_return_values.emplace_back(it->second);
            }

            // Ensure there is only one code element, then process the code text
            // auto -> std::pair<std::multimap<std::string, const tinyxml2::XMLElement*>::iterator,
            //                   std::multimap<std::string, const tinyxml2::XMLElement*>::iterator>
            auto code_range = elements.equal_range("code");
            if (std::distance(code_range.first, code_range.second) == 1)
            {
                auto it = code_range.first;
                const char* code_text = it->second->GetText();

                if (code_text != nullptr)
                {
                    m_code = processCode(it->second->GetText());
                }
                else
                {
                    m_code = "";
                }

                elements.erase(it);
            }
            else
            {
                throw ElementException("Command requires 1 code element");
            }

            // If there are any extra elements in the map, throw an exception
            if (!elements.empty())
            {
                throw ElementException(fmt::format("Extra child element for Command on line number {}",
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
            return m_code;
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

            rv += m_code;

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

        std::string Command::processCode(const std::string& code)
        {
            std::string rv = "";
            std::list<std::string> lines;
            std::istringstream input(code);

            //Split full code string into a list of
            for (std::string line; std::getline(input, line);)
            {
                lines.push_back(line);
            }

            lines.pop_front(); // Removes text on same line as code start tag <code>
            lines.pop_back(); // Removes text on same line as code end tag </code>

            // Get the number of whitespaces at the beginning of the first line of code
            std::regex re("^(\\s+)"); // All of the whitespace starting at the beginning of the line
            std::smatch match;
            std::regex_search(lines.front(), match, re);
            size_t num_whitespace_on_first_line = match.str(1).size();

            for (std::string& line : lines)
            {
                // Get the number of whitespaces at beginning of this line of code
                std::regex_search(line, match, re);
                size_t num_whitespace_on_this_line = match.str(1).size();

                // Select number of whitespaces to remove
                size_t num_whitespace = (num_whitespace_on_this_line < num_whitespace_on_first_line) ?
                                         num_whitespace_on_this_line : num_whitespace_on_first_line;

                // Make a regex string to pull out the code
                std::string reg = fmt::format("^\\s{{{num_whitespace}}}(.*?)\\s*$",
                                              fmt::arg("num_whitespace", num_whitespace));
                std::regex_search(line, match, std::regex(reg));

                rv += "\t" + match.str(1) + "\n";
            }

            return rv;
        }
    }
}
