#ifndef ARGUMENT_HPP
#define ARGUMENT_HPP

#include <string>
#include <vector>
#include <memory>

#include "xml_element.hpp"

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
         * @class Argument
         * @brief
         */
        class Argument : private XmlElement
        {
        public:

            /**
             * @brief Default Constructor
             */
            Argument();
            ~Argument();

            /**
             * @brief Constructor
             *
             * @param xml The xml element to parse for this argument
             */
            Argument(const tinyxml2::XMLElement* xml);

            /**
             * @brief Creates the code for a single argument in a constructor
             * @param  appendage The appendage used for the single constructor
             * @return The code for a single argument in a constructor
             */
            std::string toString(std::shared_ptr<Appendage> appendage) const;

            const std::string& getName() const;

            const std::string& getType() const;

            const std::string& getValue() const;

        private:
            std::string m_name;
            std::string m_type;
            std::string m_value;
        };
    } // arduinogen
}

#endif // ARGUMENT_HPP
