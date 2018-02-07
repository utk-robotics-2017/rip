#include "arduino_gen/code.hpp"

#include <sstream>
#include <regex>
#include <list>
#include <string>
#include <iostream>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Code::Code(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            m_code = processCode(getText());

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Code on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Code on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string Code::getCode() const
        {
            return m_code;
        }

        std::string Code::processCode(const std::string& code)
        {
            std::string rv = "";
            std::list<std::string> lines;
            std::istringstream input(code);

            //Split full code string into a list of
            for (std::string line; std::getline(input, line);)
            {
                lines.push_back(line);
            }

            if (lines.size() == 0)
            {
                return code;
            }

            while (std::regex_match(lines.front().c_str(), std::regex("^\\s*$")))
            {
                lines.pop_front();
            }

            while (std::regex_match(lines.back().c_str(), std::regex("^\\s*$")))
            {
                lines.pop_back();
            }

            // Get the number of whitespaces at the beginning of the first line of code
            std::regex re("^(\\s*)"); // All of the whitespace starting at the beginning of the line
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
