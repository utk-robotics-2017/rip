#include "arduino_gen/utils.hpp"

namespace rip
{
    namespace arduinogen
    {
        void replace(std::string& base, const std::string& replacee, const std::string& replacer)
        {
            base.replace(base.find(replacee), replacee.length(), replacer);
        }
    }
}
