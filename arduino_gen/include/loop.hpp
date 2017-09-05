#ifndef LOOP_HPP
#define LOOP_HPP

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
        class Loop
        {
        public:
            /**
             * @brief Constructor
             * @param xml
             */
            Loop(tinyxml2::XMLElement* xml);

            /**
             * Creates the part of the loop code for this appendage
             * @return [description]
             */
            std::string toString(std::vector< std::shared_ptr<Appendage> > appendages);

        private:
            std::string m_code;
        };
    } // namespace arduinogen
}
#endif // LOOP_HPP