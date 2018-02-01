#include "arduino_gen/code.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/xml_utils.hpp"

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
            TEST(Code_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/extra_attribute.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                ASSERT_THROW(code = std::unique_ptr<Code>(new Code(codeElement)), AttributeException);
            }

            TEST(Code_getCode, one_line)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/one_line.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t// This is on its own line\n");
            }

            TEST(Code_getCode, two_lines)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/two_lines.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t// First line\n"
                                           "\t// Second line\n");
            }

            TEST(Code_getCode, second_line_indented)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/second_line_indented.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t// First line\n"
                                           "\t    //Second line\n");
            }

            TEST(Code_getCode, first_line_indented)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/first_line_indented.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t// First line\n"
                                           "\t// Second line\n");
            }

            TEST(Code_getCode, inline)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/inline.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t// inline\n");
            }

            TEST(Code_getCode, unescaped_characters)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/code/unescaped_characters.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* codeElement = doc.FirstChildElement("code");
                ASSERT_NE(codeElement, nullptr);

                std::unique_ptr<Code> code;
                RIP_ASSERT_NO_THROW(code = std::unique_ptr<Code>(new Code(codeElement)));

                ASSERT_EQ(code->getCode(), "\t\" & ' < >\n");
            }

            /**
             * Code elements shouldn't have any child elements.
             * This is because rip::arduinogen::loadXmlFile(doc, "file.xml", {"code"})
             * is intended to escape the contents of code elements.
             */
        }
    }
}
