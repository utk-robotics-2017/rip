#ifndef INCLUDES_HPP
#define INCLUDES_HPP

#include "xml_element.hpp"

namespace rip
{
    namespace arduinogen
    {
        class Includes : private XmlElement
        {
        public:
            Includes();
            ~Includes();

            Includes(const tinyxml2::XMLElement* xml);

            std::vector<std::string> GetIncludes();

        private:
            std::vector<std::string> m_includes;
        };
    }
}

#endif //INCLUDES_HPP
