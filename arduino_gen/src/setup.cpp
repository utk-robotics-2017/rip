#include "arduino_gen/setup.hpp"

#include <regex>
#include <set>
#include <iostream>

#include <tinyxml2.h>
#include <fmt/format.h>

#include "arduino_gen/appendage.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/utils.hpp"

namespace rip
{
    namespace arduinogen
    {
        Setup::Setup(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            std::vector<const tinyxml2::XMLElement*> codes = getElements("code");

            for (const tinyxml2::XMLElement*& code_element : codes)
            {
                m_codes.emplace_back(std::make_shared<Code>(code_element));
            }

            if (m_codes.size() == 0)
            {
                throw ElementException(fmt::format("Setup must have at least one code element. Line number {}.", xml->GetLineNum()));
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

        std::string Setup::toString(std::vector< std::shared_ptr<Appendage> > appendages) const
        {
            std::string rv = "";
            std::regex replace_regex("\\$(\\w+)\\$");

            for (int i = 0; i < m_codes.size(); i++)
            {
                std::shared_ptr<Code> code = m_codes[i];

                if (code->getInsert() == "once")
                {
                    std::string appendage_setup = code->getCode();

                    std::set<std::string> matches = fill_regex_set(
                        appendage_setup, replace_regex,
                        [] (std::smatch match) { return match.str(1); });

                    for(const std::string& match : matches)
                    {
                        if (match != "n")
                        {
                            throw IllegalPatternException(fmt::format("Code element with insert cannot have a variable of ${}$. Line number {}.", match, getLineNum()));
                        }

                        std::string replacee = fmt::format("${}$", match);
                        std::size_t str_iter = appendage_setup.find(replacee);

                        while(str_iter != std::string::npos)
                        {
                            appendage_setup.replace(str_iter, replacee.length(), std::to_string(appendages.size()));
                            str_iter = appendage_setup.find(replacee);
                        }
                    }

                    rv += appendage_setup;
                }
                else if (code->getInsert() == "each")
                {
                    for (size_t i = 0; i < appendages.size(); i++)
                    {
                        std::shared_ptr<Appendage> appendage = appendages[i];

                        std::string appendage_setup = code->getCode();

                        std::set<std::string> matches = fill_regex_set(
                            appendage_setup, replace_regex,
                            [] (std::smatch match) { return match.str(1); });

                        for(const std::string& match: matches)
                        {
                            std::string replacee = fmt::format("${}$", match);
                            std::size_t str_iter = appendage_setup.find(replacee);

                            std::string value = "";

                            if (match == "i")
                            {
                                value = std::to_string(i);
                            }
                            else if (match == "n")
                            {
                                value = std::to_string(appendages.size());
                            }
                            else if (appendage->has(match))
                            {
                                value = appendage->getString(match);
                            }
                            else
                            {
                                throw PatternNotFoundException(fmt::format("Appendage \"{}\" does not have the variable \"{}\"",
                                    appendage->getType(), match));
                            }

                            while(str_iter != std::string::npos)
                            {
                                appendage_setup.replace(str_iter, replacee.length(), value);
                                str_iter = appendage_setup.find(replacee);
                            }
                        }

                        rv += appendage_setup;
                    }
                }
            }

            return rv;
        }
    } // arduinogen
}
