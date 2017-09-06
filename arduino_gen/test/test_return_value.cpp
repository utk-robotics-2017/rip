#include "return_value.hpp"
#include "exceptions.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using ReturnValue = rip::arduinogen::ReturnValue;
using AttributeException = rip::arduinogen::AttributeException;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(ReturnValue_constructor, no_attributes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/no_attributes.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                ASSERT_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)), AttributeException);
            }

            TEST(ReturnValue_constructor, one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/one_name.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                ASSERT_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)), AttributeException);
            }

            TEST(ReturnValue_constructor, one_type)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/one_type.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                ASSERT_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)), AttributeException);
            }

            TEST(ReturnValue_constructor, one_name_one_type)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/one_name_one_type.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));
            }

            TEST(ReturnValue_constructor, two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(ReturnValue_constructor, two_types)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/two_types.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(ReturnValue_constructor, one_type_one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/one_type_one_name.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));
            }

            TEST(ReturnValue_constructor, two_types_one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/two_types_one_name.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(ReturnValue_constructor, one_type_two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/one_type_two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(ReturnValue_constructor, two_types_two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/two_types_two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(ReturnValue_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/extra_attribute.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                ASSERT_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)), AttributeException);
            }

            TEST(ReturnValue_constructor, unicode)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/unicode.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));

                ASSERT_EQ(retval->getType(), "exception");
                ASSERT_EQ(retval->getName(), "╭∩╮(◣_◢)╭∩╮");
            }

            TEST(ReturnValue_get_set, monty_python)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/monty_python.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));

                ASSERT_EQ(retval->getType(), "monty");
                ASSERT_EQ(retval->getName(), "python");

                retval->setType("coconut");
                retval->setName("horse");

                ASSERT_EQ(retval->getType(), "coconut");
                ASSERT_EQ(retval->getName(), "horse");
            }

            TEST(ReturnValue_declare_send, int_var)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/int_var.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));

                ASSERT_EQ(retval->getType(), "int");
                ASSERT_EQ(retval->getName(), "var");

                ASSERT_EQ(retval->declare(), "\tint var;\n");
                ASSERT_EQ(retval->send(), "\tcmdMessenger.sendBinArg(var);\n");
            }

            TEST(ReturnValue_declare_send, char_thing)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/return_value/char_thing.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* returnValueElement = doc.FirstChildElement("return-value");
                ASSERT_NE(returnValueElement, nullptr);

                std::unique_ptr<ReturnValue> retval;
                RIP_ASSERT_NO_THROW(retval = std::unique_ptr<ReturnValue>(new ReturnValue(returnValueElement)));

                ASSERT_EQ(retval->getType(), "char");
                ASSERT_EQ(retval->getName(), "thing");

                ASSERT_EQ(retval->declare(), "\tchar thing;\n");
                ASSERT_EQ(retval->send(), "\tcmdMessenger.sendBinArg(thing);\n");
            }
        }
    }
}
