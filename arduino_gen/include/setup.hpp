#ifndef SETUP_HPP
#define SETUP_HPP

#include <string>
#include <vector>
#include <memory>

namespace tinyxml2
{
    class XMLElement;
}

namespace arduinogen
{
    class Appendage;

    /**
     * @class Setup
     * @brief The container for the setup part of
     */
    class Setup
    {
    public:
        /**
         * @brief Constructor
         */
        Setup(tinyxml2::XMLElement* xml);

        /**
         * Create the code for the part of the setup function for this appendage type
         *
         * @param appendages The list of appendages of a single type
         * 
         * @return The code for the part of the setup function for this appendage type
         */
        std::string toString(std::vector< std::shared_ptr<Appendage> > appendages) const ;

    private:
        std::string m_code;
    };
} // arduinogen

#endif 