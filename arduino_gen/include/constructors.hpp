#ifndef CONSTRUCTORS_HPP
#define CONSTRUCTORS_HPP

#include <string>
#include <vector>
#include <memory>

namespace tinyxml2
{
    class XMLElement;
} // tinyxml2

namespace rip
{
    namespace arduinogen
    {
        class Argument;
        class Appendage;

        /**
         * @class Constructors
         * @brief A container for the constructors part of the arduino code for this type
         */
        class Constructors
        {
        public:
            /**
             * @brief Constructor
             *
             * @param xml The xml element to parse
             */
            Constructors(tinyxml2::XMLElement* xml);

            /**
             * @brief Returns the arduino code for these constructors
             * @param  appendages The appendages of this type
             * @return The code for these constructors
             */
            std::string toString(std::vector< std::shared_ptr<Appendage> >& appendages) const;

        private:
            std::vector< Argument > m_arguments;

            bool m_exists;
            std::string m_type;
            std::string m_variable;
        };
    }
} // arduinogen

#endif // CONSTRUCTORS_HPP