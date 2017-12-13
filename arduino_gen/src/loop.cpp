#include "loop.hpp"

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
        Loop::Loop(tinyxml2::XMLElement* xml)
        : Code(xml)
        {
        }

        std::string Loop::toString(std::vector< std::shared_ptr<Appendage> > appendages)
        {
            std::string appendage_loop = getCode();
            std::regex replace_regex("\\$(\\w+)\\$");

            for(std::shared_ptr<Appendage> appendage : appendages)
            {
                std::sregex_iterator reg_begin = std::sregex_iterator(appendage_loop.begin(), appendage_loop.end(),
                                                 replace_regex);
                std::sregex_iterator reg_end = std::sregex_iterator();

                std::set< std::string > matches;

                for(std::sregex_iterator reg_iter = reg_begin; reg_iter != reg_end; ++reg_iter)
                {
                    std::smatch match = *reg_iter;
                    std::string match_str = match.str(1);
                    matches.insert(match_str);
                }

                for(const std::string& match: matches)
                {
                    std::string replacee = fmt::format("${}$", match);
                    std::size_t str_iter = appendage_loop.find(replacee);

                    if (!appendage->has(match))
                    {
                        throw PatternNotFoundException(fmt::format("Appendage \"{}\" does not have the variable \"{}\"",
                            appendage->getType(), match));
                    }

                    while(str_iter != std::string::npos)
                    {
                        appendage_loop.replace(str_iter, replacee.length(), appendage->getString(match));
                        str_iter = appendage_loop.find(replacee);
                    }
                }
            }

            return appendage_loop;
        }
    } // namespace arduinogen
}
