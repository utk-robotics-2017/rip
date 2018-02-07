#include "arduino_gen/xml_element.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        XmlElement::XmlElement() = default;
        XmlElement::~XmlElement() = default;

        XmlElement::XmlElement(const tinyxml2::XMLElement* xml) : tinyElement(xml)
        {
            // Loop through the attributes in the element and add them to the map
            // auto -> const tinyxml2::XMLAttribute*
            for (auto attr = tinyElement->FirstAttribute(); attr != nullptr; attr = attr->Next())
            {
                m_attributes.emplace(attr->Name(), attr);
            }

            // Loop through all of the child nodes, get their type, and store them in the containers
            // auto -> const tinyxml2::XMLNode*
            for (auto child = tinyElement->FirstChild(); child != nullptr; child = child->NextSibling())
            {
                const tinyxml2::XMLElement* element = child->ToElement();
                if (element != nullptr)
                {
                    m_elements.emplace(element->Name(), element);
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLElement, element));
                    continue;
                }

                const tinyxml2::XMLText* text = child->ToText();
                if (text != nullptr)
                {
                    m_texts.emplace_back(text);
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLText, text));
                    continue;
                }

                const tinyxml2::XMLComment* comment = child->ToComment();
                if (comment != nullptr)
                {
                    m_comments.emplace_back(comment);
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLComment, comment));
                    continue;
                }

                const tinyxml2::XMLDeclaration* declaration = child->ToDeclaration();
                if (declaration != nullptr)
                {
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLDeclaration, declaration));
                    continue;
                }

                const tinyxml2::XMLDocument* document = child->ToDocument();
                if (document != nullptr)
                {
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLDocument, document));
                    continue;
                }

                const tinyxml2::XMLUnknown* unknown = child->ToUnknown();
                if (unknown != nullptr)
                {
                    m_children.emplace_back(std::make_pair(XMLNodeType::XMLUnknown, unknown));
                    continue;
                }
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

        std::vector<std::string> XmlElement::getTexts() const
        {
            std::vector<std::string> texts;

            for (auto text : m_texts)
            {
                texts.emplace_back(text->Value());
            }

            return texts;
        }

        std::string XmlElement::getText() const
        {
            std::string rv = "";

            for (auto text : m_texts)
            {
                rv.append(text->Value());
            }

            return rv;
        }

        std::vector<std::string> XmlElement::getComments() const
        {
            std::vector<std::string> comments;

            for (auto comment : m_comments)
            {
                comments.emplace_back(comment->Value());
            }

            return comments;
        }

        std::vector<std::pair<XMLNodeType, const tinyxml2::XMLNode*>> XmlElement::getChildren() const
        {
            return m_children;
        }

        int XmlElement::getLineNum() const
        {
            return tinyElement->GetLineNum();
        }
    }
}
