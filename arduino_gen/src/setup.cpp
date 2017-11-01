#include "setup.hpp"

#include <regex>
#include <set>

#include <tinyxml2.h>
#include <fmt/format.h>

#include "appendage.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        Setup::Setup(tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            // Gets all of the parameter child elements, construct Parameters and store in the vector
            for (const tinyxml2::XMLElement* ele : getElements("parameter"))
            {
                m_parameters.emplace_back(ele);
            }

            // Ensure there is only one code element, then process the code text
            std::vector<const tinyxml2::XMLElement*> code_elements = getElements("code");
            if (code_elements.size() == 1)
            {
                m_code = std::unique_ptr<Code>(new Code(code_elements[0]));
            }
            else
            {
                throw ElementException("Setup requires 1 code element");
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for Setup on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for Setup on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::string Setup::toString(std::vector< std::shared_ptr<Appendage> > appendages) const
        {
            std::string rv;

            std::regex replace_regex("{{(\\w+)}}");

            for(std::shared_ptr<Appendage> appendage : appendages)
            {
                std::string appendage_setup = m_code->getCode();
                std::sregex_iterator reg_begin = std::sregex_iterator(appendage_setup.begin(),
                                                 appendage_setup.end(), replace_regex);
                std::sregex_iterator reg_end = std::sregex_iterator();

                std::set< std::string > matches;

                for(std::sregex_iterator reg_iter = reg_begin; reg_iter != reg_end; ++reg_iter)
                {
                    std::smatch match = *reg_iter;
                    std::string match_str = match.str();
                    matches.insert(match_str);
                }

                for(const std::string& match: matches)
                {
                    std::string replacee = fmt::format("{{{{{}}}}}", match);
                    std::size_t str_iter = appendage_setup.find(replacee);
                    while(str_iter != std::string::npos)
                    {
                        appendage_setup.replace(str_iter, replacee.length(), appendage->getString(match));
                        str_iter = appendage_setup.find(replacee);
                    }
                }
                rv += appendage_setup;
            }

            return rv;
        }
    } // arduinogen
}
