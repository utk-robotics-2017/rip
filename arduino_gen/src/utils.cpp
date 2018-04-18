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
    }
}
