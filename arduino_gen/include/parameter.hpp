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
            Parameter(const tinyxml2::XMLElement* xml, std::string id);

            /**
             *  @brief Creates the code for recieving the parameter in a command's callback function
             */
            std::string receive() const;

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

            /**
             * @brief Getter for `m_id`
             * @returns m_id
             */
            std::string getId() const;

            /**
             * @brief Setter for `m_id`
             * @param The new id
             */
            void setId(const std::string& id);

        private:
            std::string m_name;
            std::string m_type;
            std::string m_id;
        };
    }
}

#endif // PARAMETER_HPP
