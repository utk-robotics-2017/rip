#include "arduino_gen/parameter.hpp"
#include "arduino_gen/exceptions.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Parameter = rip::arduinogen::Parameter;
using AttributeException = rip::arduinogen::AttributeException;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Parameter_constructor, no_attributes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/no_attributes.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                ASSERT_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")), AttributeException);
            }

            TEST(Parameter_constructor, one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/one_name.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                ASSERT_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")), AttributeException);
            }

            TEST(Parameter_constructor, one_type)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/one_type.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                ASSERT_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")), AttributeException);
            }

            TEST(Parameter_constructor, one_name_one_type)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/one_name_one_type.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "bravo");
                ASSERT_EQ(param->getName(), "alpha");
            }

            TEST(Parameter_constructor, two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(Parameter_constructor, two_types)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/two_types.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(Parameter_constructor, one_type_one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/one_type_one_name.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "honk");
                ASSERT_EQ(param->getName(), "blarg");
            }

            TEST(Parameter_constructor, two_types_one_name)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/two_types_one_name.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(Parameter_constructor, one_type_two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/one_type_two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(Parameter_constructor, two_types_two_names)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/two_types_two_names.xml"), tinyxml2::XML_ERROR_PARSING_ATTRIBUTE);
            }

            TEST(Parameter_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/extra_attribute.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                ASSERT_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")), AttributeException);
            }

            TEST(Parameter_constructor, unicode)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/unicode.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "exception");
                ASSERT_EQ(param->getName(), "╭∩╮(◣_◢)╭∩╮");
            }

            TEST(Parameter_get_set, monty_python)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/monty_python.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "monty");
                ASSERT_EQ(param->getName(), "python");
                ASSERT_EQ(param->getId(), "TestId");

                param->setType("one");
                param->setName("two");
                param->setId("five");

                ASSERT_EQ(param->getType(), "one");
                ASSERT_EQ(param->getName(), "two");
                ASSERT_EQ(param->getId(), "five");
            }

            TEST(Parameter_receive, int_var)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/int_var.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "int");
                ASSERT_EQ(param->getName(), "var");
                ASSERT_EQ(param->getId(), "TestId");

                std::string expected_string =
                    "\tint var = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, TestId);\n"
                    "\t\treturn;\n"
                    "\t}\n";

                ASSERT_EQ(param->receive(), expected_string);
            }

            TEST(Parameter_receive, char_thing)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/parameter/char_thing.xml"), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* parameterElement = doc.FirstChildElement("parameter");
                ASSERT_NE(parameterElement, nullptr);

                std::unique_ptr<Parameter> param;
                RIP_ASSERT_NO_THROW(param = std::unique_ptr<Parameter>(new Parameter(parameterElement, "TestId")));

                ASSERT_EQ(param->getType(), "char");
                ASSERT_EQ(param->getName(), "thing");
                ASSERT_EQ(param->getId(), "TestId");

                std::string expected_string =
                    "\tchar thing = cmdMessenger.readBinArg<char>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, TestId);\n"
                    "\t\treturn;\n"
                    "\t}\n";

                ASSERT_EQ(param->receive(), expected_string);
            }
        }
    }
}
