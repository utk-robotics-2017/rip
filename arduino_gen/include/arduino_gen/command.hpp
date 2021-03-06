#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <string>
#include <vector>
#include <memory>

#include "xml_element.hpp"
#include "code.hpp"
#include "parameter.hpp"
#include "return_value.hpp"

namespace tinyxml2
{
    class XMLElement;
}

namespace rip
{
    namespace arduinogen
    {
        /**
         * @class Command
         * @brief Contains the parsed information for a single command
         */
        class Command : private XmlElement
        {
        public:

            /**
             * @brief Default Constructor
             *
             * @note Only for the constructor
             */
            //Command() = default;

            /**
             * @brief Constructor
             */
            Command(const tinyxml2::XMLElement* command);

            /*
            Command(const Command& other) = delete;
            Command(Command&&) = default;
            Command& operator=(const Command& other) = delete;
            Command& operator=(Command&&) & = default;
            */

            /**
             * @brief Gets the enum id for the CmdMessenger
             */
            std::string getId() const;

            /**
             * @brief Gets the enum id for the result (empty if there is no result)
             *
             * @note The string should be the id of this command contcatenated with
             *       "Result" if there is a result
             */
            std::string getResultId() const;

            /**
             * @brief Gets the callback function name
             */
            std::string getName() const;

            /**
             * @brief Gets whether the callback function should collect the index number
             */
            bool getIndexNum() const;

            /**
             * @brief Gets the code used in between receiving parameters and sending the response
             */
            std::string getCode() const;

            /**
             * @brief Creates the callback function
             *
             * @param num_appendages The number of appendages of this type
             */
            std::string callback(int num_appendages) const;

        private:
            std::string m_id;
            std::string m_name;
            bool m_index_num;
            std::vector<Parameter> m_parameters;
            std::vector<ReturnValue> m_return_values;
            std::unique_ptr<Code> m_code;
        };
    }
}

#endif // COMMAND_HPP
