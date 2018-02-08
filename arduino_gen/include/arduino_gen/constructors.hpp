#ifndef CONSTRUCTORS_HPP
#define CONSTRUCTORS_HPP

#include "constructor.hpp"
#include "xml_element.hpp"

#include <vector>

namespace rip
{
    namespace arduinogen
    {
        class Constructors : private XmlElement
        {
        public:
            /**
             * @brief Constructor
             *
             * @param xml The xml element to parse
             */
            Constructors(const tinyxml2::XMLElement* xml);

            /**
             * @brief Returns the arduino code for these constructors
             * @param  appendages The appendages of this type
             * @return The code for these constructors
             */
            std::string toString(const std::vector<std::shared_ptr<Appendage>>& appendages) const;

        private:
            std::vector<Constructor> m_constructors;
        };
    }
}

#endif // CONSTRUCTORS_HPP
