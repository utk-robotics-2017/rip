#include "command.hpp"
#include "exceptions.hpp"
#include "xml_utils.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Command = rip::arduinogen::Command;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Command_constructor, no_attributes_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/no_attributes_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, id_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, name_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/name_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, id_name_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), ElementException);
            }

            TEST(Command_constructor, index_num_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/index_num_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, id_index_num_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_index_num_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, name_index_num_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/name_index_num_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, id_name_index_num_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), ElementException);
            }

            TEST(Command_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/extra_attribute.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), AttributeException);
            }

            TEST(Command_constructor, extra_element)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/extra_element.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                ASSERT_THROW(command = std::unique_ptr<Command>(new Command(commandElement)), ElementException);
            }

            TEST(Command_constructor, id_name_empty_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_empty_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));
            }

            TEST(Command_callback, id_name_code)
            {
                tinyxml2::XMLDocument doc(true, tinyxml2::PRESERVE_WHITESPACE);
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "No");
                ASSERT_EQ(command->getResultId(), "");
                ASSERT_EQ(command->getName(), "Exceptions");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\t// Proper indentation is semi-important\n"
                    "\t// Even on the second line\n");
                ASSERT_EQ(command->callback(1),
                    "void Exceptions() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, No);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\t// Proper indentation is semi-important\n"
                    "\t// Even on the second line\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, No);\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_true_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_true_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "Easter");
                ASSERT_EQ(command->getResultId(), "");
                ASSERT_EQ(command->getName(), "Egg");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\t// All i'm saying, is that I need an index-num\n");
                ASSERT_EQ(command->callback(1),
                    "void Egg() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, Easter);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\t// All i'm saying, is that I need an index-num\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, Easter);\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_false_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_false_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "Not");
                ASSERT_EQ(command->getResultId(), "");
                ASSERT_EQ(command->getName(), "Picky");
                ASSERT_EQ(command->getIndexNum(), false);
                ASSERT_EQ(command->getCode(),
                    "\t// That other guy is too needy, I don't need an index-num\n");
                ASSERT_EQ(command->callback(1),
                    "void Picky() {\n"
                    "\t// That other guy is too needy, I don't need an index-num\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, Not);\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_one_parameter_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_one_parameter_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "Jurassic");
                ASSERT_EQ(command->getResultId(), "");
                ASSERT_EQ(command->getName(), "Park");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\t// Got your nose!\n");
                ASSERT_EQ(command->callback(1),
                    "void Park() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, Jurassic);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tVelociraptor CleverGirl = cmdMessenger.readBinArg<Velociraptor>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, Jurassic);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\t// Got your nose!\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, Jurassic);\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_two_parameters_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_two_parameters_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "ACleverId");
                ASSERT_EQ(command->getResultId(), "");
                ASSERT_EQ(command->getName(), "ACleverName");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\t// I should probably do something with \"a\" and \"b\"\n");
                ASSERT_EQ(command->callback(1),
                    "void ACleverName() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, ACleverId);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint a = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, ACleverId);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint b = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, ACleverId);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\t// I should probably do something with \"a\" and \"b\"\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, ACleverId);\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_one_return_value_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_one_return_value_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "Euphoria");
                ASSERT_EQ(command->getResultId(), "EuphoriaResult");
                ASSERT_EQ(command->getName(), "Cloud9");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\tcloud = 9;\n");
                ASSERT_EQ(command->callback(1),
                    "void Cloud9() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, Euphoria);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint cloud;\n"
                    "\tcloud = 9;\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, Euphoria);\n"
                    "\tcmdMessenger.sendCmdStart(EuphoriaResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(cloud);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_two_return_values_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_two_return_values_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "ThreeDog");
                ASSERT_EQ(command->getResultId(), "ThreeDogResult");
                ASSERT_EQ(command->getName(), "Night");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\tTheLoneliestNumber = 1;\n"
                    "\tNotAsBadAsOne = 2;\n");
                ASSERT_EQ(command->callback(1),
                    "void Night() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, ThreeDog);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint TheLoneliestNumber;\n"
                    "\tint NotAsBadAsOne;\n"
                    "\tTheLoneliestNumber = 1;\n"
                    "\tNotAsBadAsOne = 2;\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, ThreeDog);\n"
                    "\tcmdMessenger.sendCmdStart(ThreeDogResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(TheLoneliestNumber);\n"
                    "\tcmdMessenger.sendCmdBinArg(NotAsBadAsOne);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_one_parameter_one_return_value_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_one_parameter_one_return_value_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "kEcho");
                ASSERT_EQ(command->getResultId(), "kEchoResult");
                ASSERT_EQ(command->getName(), "Echo");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\tout = in;\n");
                ASSERT_EQ(command->callback(1),
                    "void Echo() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kEcho);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tchar in = cmdMessenger.readBinArg<char>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kEcho);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tchar out;\n"
                    "\tout = in;\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, kEcho);\n"
                    "\tcmdMessenger.sendCmdStart(kEchoResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(out);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_two_parameters_one_return_value_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_two_parameters_one_return_value_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "kExponent");
                ASSERT_EQ(command->getResultId(), "kExponentResult");
                ASSERT_EQ(command->getName(), "XToTheN");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\trv = 1;\n"
                    "\tfor (int i = 0; i < n; i++)\n"
                    "\t    rv *= x;\n");
                ASSERT_EQ(command->callback(1),
                    "void XToTheN() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kExponent);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint x = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kExponent);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint n = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kExponent);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint rv;\n"
                    "\trv = 1;\n"
                    "\tfor (int i = 0; i < n; i++)\n"
                    "\t    rv *= x;\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, kExponent);\n"
                    "\tcmdMessenger.sendCmdStart(kExponentResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(rv);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_one_parameter_two_return_values_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_one_parameter_two_return_values_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "kOneToTwo");
                ASSERT_EQ(command->getResultId(), "kOneToTwoResult");
                ASSERT_EQ(command->getName(), "OneToTwo");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\t// Make two outputs from one input\n");
                ASSERT_EQ(command->callback(1),
                    "void OneToTwo() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kOneToTwo);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint a = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kOneToTwo);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint rv1;\n"
                    "\tint rv2;\n"
                    "\t// Make two outputs from one input\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, kOneToTwo);\n"
                    "\tcmdMessenger.sendCmdStart(kOneToTwoResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(rv1);\n"
                    "\tcmdMessenger.sendCmdBinArg(rv2);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }

            TEST(Command_callback, id_name_index_num_two_parameters_two_return_values_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/command/id_name_index_num_two_parameters_two_return_values_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* commandElement = doc.FirstChildElement("command");
                ASSERT_NE(commandElement, nullptr);

                std::unique_ptr<Command> command;
                RIP_ASSERT_NO_THROW(command = std::unique_ptr<Command>(new Command(commandElement)));

                ASSERT_EQ(command->getId(), "kDivide");
                ASSERT_EQ(command->getResultId(), "kDivideResult");
                ASSERT_EQ(command->getName(), "Divide");
                ASSERT_EQ(command->getIndexNum(), true);
                ASSERT_EQ(command->getCode(),
                    "\tquotient = a / b;\n"
                    "\tremainder = a % b;\n");
                ASSERT_EQ(command->callback(1),
                    "void Divide() {\n"
                    "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kDivide);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint a = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kDivide);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint b = cmdMessenger.readBinArg<int>();\n"
                    "\tif(!cmdMessenger.isArgOk()) {\n"
                    "\t\tcmdMessenger.sendBinCmd(kError, kDivide);\n"
                    "\t\treturn;\n"
                    "\t}\n"
                    "\tint quotient;\n"
                    "\tint remainder;\n"
                    "\tquotient = a / b;\n"
                    "\tremainder = a % b;\n"
                    "\tcmdMessenger.sendBinCmd(kAcknowledge, kDivide);\n"
                    "\tcmdMessenger.sendCmdStart(kDivideResult);\n"
                    "\tcmdMessenger.sendCmdBinArg(quotient);\n"
                    "\tcmdMessenger.sendCmdBinArg(remainder);\n"
                    "\tcmdMessenger.sendCmdEnd();\n"
                    "}\n");
            }
        }
    }
}
