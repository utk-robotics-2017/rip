#include "arduino_gen/utils.hpp"

namespace rip
{
    namespace arduinogen
    {
        void replace(std::string& base, const std::string& replacee, const std::string& replacer)
        {
            base.replace(base.find(replacee), replacee.length(), replacer);
        }

        std::vector<std::string> split(const std::string &s, char delim) {
            std::vector<std::string> elems;
            split(s, delim, std::back_inserter(elems));
            return elems;
        }

        std::set<std::string> fill_regex_set(const std::string& str, const std::regex& reg, std::function<std::string(std::smatch)> selector)
        {
            std::set<std::string> matches;
            fill_regex(str, reg, selector, std::inserter(matches, end(matches)));
            return matches;
        }

        std::vector<std::string> fill_regex_vector(const std::string& str, const std::regex& reg, std::function<std::string(std::smatch)> selector)
        {
            std::vector<std::string> matches;
            fill_regex(str, reg, selector, std::back_inserter(matches));
            return matches;
        }
    }
}
