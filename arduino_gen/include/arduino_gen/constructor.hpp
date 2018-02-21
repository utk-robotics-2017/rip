#ifndef CONSTRUCTOR_HPP
#define CONSTRUCTOR_HPP

#include <string>
#include <vector>
#include <memory>

#include "xml_element.hpp"
#include "argument.hpp"

namespace tinyxml2
{
    class XMLElement;
} // tinyxml2

namespace rip
{
    namespace arduinogen
    {
        class Appendage;

        /**
         * @class Constructors
         * @brief A container for the constructors part of the arduino code for this type
         */
        class Constructor : private XmlElement
        {
        public:
            /**
             * @brief Constructor
             *
             * @param xml The xml element to parse
             */
            Constructor(const tinyxml2::XMLElement* xml);

            /**
             * @brief Returns the arduino code for these constructors
             * @param  appendages The appendages of this type
             * @return The code for these constructors
             */
            std::string toString(const std::vector<std::shared_ptr<Appendage>>& appendages) const;

        private:
            std::vector<Argument> m_arguments;

            std::string m_type;
            std::string m_variable;
            bool m_type_is_class;
        };
    } // arduinogen
}

#endif // CONSTRUCTOR_HPP
