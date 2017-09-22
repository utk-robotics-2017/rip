#include "code.hpp"
#include "exceptions.hpp"
#include "xml_utils.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Code = rip::arduinogen::Code;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Code_constructor, one_line)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/one_line.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t// This is on its own line\n");
            }

            TEST(Code_constructor, two_lines)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/two_lines.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t// First line\n"
                                           "\t// Second line\n");
            }

            TEST(Code_constructor, second_line_indented)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/second_line_indented.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t// First line\n"
                                           "\t    //Second line\n");
            }

            TEST(Code_constructor, first_line_indented)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/first_line_indented.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t// First line\n"
                                           "\t// Second line\n");
            }

            TEST(Code_constructor, inline)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/inline.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t// inline\n");
            }

            TEST(Code_constructor, unescaped_characters)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/unescaped_characters.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getText(), "\t\" & ' < >\n");
            }

            TEST(Code_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/extra_attribute.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                ASSERT_THROW(code = std::unique_ptr<Code>(new Code(codeElement)), AttributeException);
            }

            TEST(Code_constructor, extra_element)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/extra_element.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                ASSERT_THROW(code = std::unique_ptr<Code>(new Code(codeElement)), ElementException);
            }

            TEST(Code_constructor, line_extra_element)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/line_extra_element.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                ASSERT_THROW(code = std::unique_ptr<Code>(new Code(codeElement)), ElementException);
            }

            TEST(Code_constructor, extra_element_line)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/extra_element_line.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                ASSERT_THROW(code = std::unique_ptr<Code>(new Code(codeElement)), ElementException);
            }
        }
    }
}
