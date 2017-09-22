#ifndef XMLELEMENT_HPP
#define XMLELEMENT_HPP

#include <map>
#include <vector>
#include <string>

namespace tinyxml2
{
    class XMLElement;
    class XMLAttribute;
}

namespace rip
{
    namespace arduinogen
    {
        class XmlElement
        {
        public:
            /**
             * @brief Takes the tinyxml2 element and extracts pointers to the attributes and elements
             *
             * @param xml The tinyxml2 element to be converted
             */
            XmlElement(const tinyxml2::XMLElement* xml);

            /**
             * @brief Gets the attribute from the map, then erases it from the map
             *
             * @param name The name of the attribute to get
             *
             * @throws AttributeException The key `name` is not in the attributes map
             */
            const tinyxml2::XMLAttribute* getAttribute(std::string name);

            /**
             * @brief Gets the elements from the map, then erases them from the map
             *
             * @param name The name of the elements to get
             *
             * @throws ElementException The key `name` is not in the elements map
             */
            std::vector<const tinyxml2::XMLElement*> getElements(std::string name);

            /**
             * @brief Checks if the attributes map is empty
             */
            bool isAttributesEmpty() const;

            /**
             * @brief Checks if the elements map is empty
             */
            bool isElementsEmpty() const;

        private:
            const tinyxml2::XMLElement* tinyElement;
            std::map<std::string, const tinyxml2::XMLAttribute*> m_attributes;
            std::multimap<std::string, const tinyxml2::XMLElement*> m_elements;
        };
    }
}

#endif // XMLELEMENT_HPP
