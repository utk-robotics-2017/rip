#ifndef SETUP_PARAMETER_HPP
#define SETUP_PARAMETER_HPP

#include <string>

#include "xml_element.hpp"

namespace tinyxml2
{
    class XMLElement;
} // tinyxml2

namespace rip
{
    namespace arduinogen
    {
        /**
         * @class Parameter
         * @brief A single parameter for a command's callback function
         */
        class SetupParameter : private XmlElement
        {
        public:
            /**
             * @brief Default Constructor
             */
            SetupParameter() = default;

            /**
             * @brief Constructor
             *
             * @param xml The xml to parse for constructing this parameter
             * @param id The id of the command that this parameter is a part of
             */
            SetupParameter(const tinyxml2::XMLElement* xml);

            std::string getName() const;

            std::string getReplacer() const;

        private:
            std::string m_name;
            std::string m_replacer;
        };
    }
}

#endif // SETUP_PARAMETER_HPP
