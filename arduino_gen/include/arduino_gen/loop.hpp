#ifndef LOOP_HPP
#define LOOP_HPP

#include "code.hpp"

#include <string>
#include <vector>
#include <memory>

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
         * @class Loop
         * @brief
         */
        class Loop : private XmlElement
        {
        public:
            /**
             * @brief Constructor
             * @param xml
             */
            Loop(const tinyxml2::XMLElement* xml);

            Loop(const Loop& other) = delete;
            Loop& operator=(const Loop& other) = delete;

            /**
             * Creates the part of the loop code for this appendage
             * @return [description]
             */
            std::string toString(std::vector< std::shared_ptr<Appendage> > appendages);

        private:
            std::vector<std::shared_ptr<Code>> m_codes;
        };
    } // namespace arduinogen
}

#endif // LOOP_HPP
