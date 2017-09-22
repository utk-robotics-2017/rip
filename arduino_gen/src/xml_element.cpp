#include "xml_element.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        XmlElement::XmlElement(const tinyxml2::XMLElement* xml) : tinyElement(xml)
        {
            // Loop through the attributes in the element and add them to the map
            // auto -> const tinyxml2::XMLAttribute*
            for (auto attr = tinyElement->FirstAttribute(); attr != nullptr; attr = attr->Next())
            {
                m_attributes.emplace(attr->Name(), attr);
            }

            // Loop through the child elements and add them to the map
            // auto -> const tinyxml2::XMLElement*
            for (auto ele = tinyElement->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
            {
                m_elements.emplace(ele->Name(), ele);
            }
        }

        const tinyxml2::XMLAttribute* XmlElement::getAttribute(std::string name)
        {
            // Get the value for the attribute, then erase it from the map
            try
            {
                // Get the attribute from the map, then erase it from the map
                const tinyxml2::XMLAttribute* attribute = m_attributes.at(name);
                m_attributes.erase(name);

                return attribute;
            }
            catch (const std::out_of_range& e)
            {
                // The key is not in the map, throw an exception
                throw AttributeException(fmt::format("Attribute {} missing on line number {}",
                                                     name,
                                                     tinyElement->GetLineNum()));
            }
        }

        std::vector<const tinyxml2::XMLElement*> XmlElement::getElements(std::string name)
        {
            // Get the value for the element, then erase it from the map
            try
            {
                std::vector<const tinyxml2::XMLElement*> elements;

                // auto -> std::pair<std::multimap<std::string, const tinyxml2::XMLElement*>::iterator,
                //                   std::multimap<std::string, const tinyxml2::XMLElement*>::iterator>
                auto elements_range = m_elements.equal_range(name);

                // Reserve space for the number of elements we're getting
                elements.reserve(std::distance(elements_range.first, elements_range.second));

                // Loop through the elements, and remove them after being processed
                for (auto it = elements_range.first; it != elements_range.second; it = m_elements.erase(it))
                {
                    elements.emplace_back(it->second);
                }

                return elements;
            }
            catch (const std::out_of_range& e)
            {
                // The key is not in the map, throw an exception
                throw ElementException(fmt::format("Element {} missing on line number {}",
                                                   name,
                                                   tinyElement->GetLineNum()));
            }
        }

        bool XmlElement::isAttributesEmpty() const
        {
            return m_attributes.empty();
        }

        bool XmlElement::isElementsEmpty() const
        {
            return m_elements.empty();
        }
    }
}
