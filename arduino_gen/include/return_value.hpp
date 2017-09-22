#ifndef RETURN_VALUE_HPP
#define RETURN_VALUE_HPP

#include <string>

#include "xml_element.hpp"

namespace tinyxml2
{
    class XMLElement;
}

namespace rip
{
    namespace arduinogen
    {
        /**
         * @class ReturnValue
         * @brief Container for the return value of a command's callback function
         */
        class ReturnValue : public XmlElement
        {
        public:
            /**
             * @brief Default Constructor
             */
            ReturnValue() = default;

            /**
             * @brief Parse Constructor
             *
             * @param xml The xml element to parse
             */
            ReturnValue(const tinyxml2::XMLElement* xml);

            /**
             * @brief Create the code for declaring the return value variable
             */
            std::string declare() const;

            /**
             * @brief Creates the code for sending the return value to RIP
             */
            std::string send() const;

            /**
             * @brief Getter for `m_name`
             * @returns m_name
             */
            std::string getName() const;

            /**
             * @brief Setter for `m_name`
             * @param The new name
             */
            void setName(const std::string& name);

            /**
             * @brief Getter for `m_type`
             * @returns m_type
             */
            std::string getType() const;

            /**
             * @brief Setter for `m_type`
             * @param The new type
             */
            void setType(const std::string& type);

        private:
            std::string m_name;
            std::string m_type;
        };
    }
}

#endif // RETURN_VALUE_HPP
