#ifndef RETURN_VALUE_HPP
#define RETURN_VALUE_HPP

#include <string>

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
        class ReturnValue
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
            ReturnValue(tinyxml2::XMLElement* xml);

            /**
             * @brief Create the code for declaring the return value variable
             */
            std::string declare() const;

            /**
             * @brief Creates the code for sending the return value to RIP
             */
            std::string send() const;
        private:
            std::string m_name;
            std::string m_type;
        };
    }
}

#endif // RETURN_VALUE_HPP
