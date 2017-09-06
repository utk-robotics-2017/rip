#include "xml_utils.hpp"

#include <fstream>
#include <sstream>
#include <regex>
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>

#include <fmt/format.h>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        tinyxml2::XMLError loadXmlFile(tinyxml2::XMLDocument& doc, const std::string& filename, const std::vector<std::string>& elements_to_escape)
        {
            std::string file, escaped, expression, suffix;

            // Read in file
            std::ifstream in(filename, std::ios::in);
            if (!in)
            {
                throw FileIoException("Error reading file");
            }
            file = std::string{std::istreambuf_iterator<char>{in}, {}};
            in.close();

            for (std::string element : elements_to_escape)
            {
                // https://regex101.com/r/ECLQuZ/1
                expression = fmt::format("(<(?:{element}).*>)((?:[\\r\\n]|.)*?)(<\\/(?:{element}).*>)",
                                         fmt::arg("element", element));
                std::regex reg(expression);
                std::sregex_iterator re_it(file.begin(), file.end(), reg), re_it_end;

                // Make sure there is a match
                if (re_it != re_it_end)
                {
                    escaped = "";

                    // Iterate through the matches
                    for (; re_it != re_it_end; ++re_it)
                    {
                        escaped += re_it->prefix().str();    // Everything before this match
                        escaped += re_it->str(1);            // Element Start Tag
                        escaped += xmlEncode(re_it->str(2)); // Element Contents
                        escaped += re_it->str(3);            // Element End Tag
                        suffix = re_it->suffix().str();      // Everything after this match
                    }
                    escaped += suffix;

                    file = escaped;
                }
            }

            return doc.Parse(file.c_str());;
        }

        std::string xmlEncode(const std::string& str)
        {
            std::string rv = "";

            for (char c : str)
            {
                switch(c)
                {
                    case '"':
                        rv += "&quot;";
                        break;
                    case '\'':
                        rv += "&apos;";
                        break;
                    case '<':
                        rv += "&lt;";
                        break;
                    case '>':
                        rv += "&gt;";
                        break;
                    case '&':
                        rv += "&amp;";
                        break;
                    default:
                        rv += c;
                        break;
                }
            }

            return rv;
        }
    }
}
