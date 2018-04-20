#ifndef CODE_HPP
#define CODE_HPP

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
         * @class Code
         * @brief The code section to be inserted into the arduino code
         */
        class Code : private XmlElement
        {
        public:
            /**
             * @brief Default Constructor
             */
            Code() = default;

            /**
             * @brief Constructor
             *
             * @param xml The xml to parse for constructing this parameter
             * @param id The id of the command that this parameter is a part of
             */
            Code(const tinyxml2::XMLElement* xml);

            std::string getCode() const;

            std::string getInsert() const;

        private:
            std::string m_code;
            std::string m_insert;

            /**
             * @brief Removes extra whitespace from the code element
             *
             * @param Raw string from the code element
             * @return formatted code
             */
            std::string processCode(const std::string& code);
        };
    }
}

#endif // CODE_HPP
