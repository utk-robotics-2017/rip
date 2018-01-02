#ifndef SETUP_HPP
#define SETUP_HPP

#include <string>
#include <vector>
#include <memory>

#include "xml_element.hpp"
#include "code.hpp"

namespace tinyxml2
{
    class XMLElement;
}

namespace rip
{
    namespace arduinogen
    {
        class Appendage;

        /**
         * @class Setup
         * @brief The container for the setup part of
         */
        class Setup : private Code
        {
        public:
            /**
             * @brief Constructor
             */
            Setup(const tinyxml2::XMLElement* xml);

            /**
             * Create the code for the part of the setup function for this appendage type
             *
             * @param appendages The list of appendages of a single type
             *
             * @return The code for the part of the setup function for this appendage type
             */
            std::string toString(std::vector< std::shared_ptr<Appendage> > appendages) const;

        private:
        };
    } // arduinogen
}

#endif
