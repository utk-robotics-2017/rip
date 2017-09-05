#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <string>

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
        class Parameter
        {
        public:
            /**
             * @brief Default Constructor
             */
            Parameter() = default;

            /**
             * @brief Constructor
             *
             * @param xml The xml to parse for constructing this parameter
             * @param id The id of the command that this parameter is a part of
             */
            Parameter(tinyxml2::XMLElement* xml, std::string id);

            /**
             *  @brief Creates the code for recieving the parameter in a command's callback function
             */
            std::string receive() const;

        private:
            std::string m_name;
            std::string m_type;
            std::string m_id;
        };
    }
}

#endif // PARAMETER_HPP
