#include "arduino_gen/loop.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/xml_utils.hpp"
#include "arduino_gen/appendage.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>
#include <iostream>

using Loop = rip::arduinogen::Loop;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Loop_constructor, no_attributes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/no_attributes.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));
            }

            TEST(Loop_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/extra_attribute.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                ASSERT_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)), AttributeException);
            }

            TEST(Loop_toString, analog_input_name_mismatch)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/analog_input_name_mismatch.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                ASSERT_THROW(loopStr = loop->toString(appendages), PatternNotFoundException);
            }

            TEST(Loop_toString, analog_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/analog_input.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr, "\tval = analogRead(1);\n");
            }

            TEST(Loop_toString, digital_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/digital_input.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Digital Input";
                j["label"] = "Whatever";
                j["pin"] = 1;
                j["pullup"] = "_PULLUP";

                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr, "\tval = digitalRead(1);\n");
            }

            TEST(Loop_toString, multi_code)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/multi_code.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Not A Real Type";
                j["label"] = "Not A Real Label";
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr,
                    "\t// Code 1\n"
                    "\t// i: 0\n"
                    "\t// n: 2\n"
                    "\t// End  1\n"
                    "\t// Code 1\n"
                    "\t// i: 1\n"
                    "\t// n: 2\n"
                    "\t// End  1\n"
                    "\t// Code 2\n"
                    "\t// i: 0\n"
                    "\t// n: 2\n"
                    "\t// End  2\n"
                    "\t// Code 2\n"
                    "\t// i: 1\n"
                    "\t// n: 2\n"
                    "\t// End  2\n"
                );
            }

            TEST(Loop_toString, multi_code_each)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/multi_code_each.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Not A Real Type";
                j["label"] = "Not A Real Label";
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr,
                    "\t// Code 1\n"
                    "\t// i: 0\n"
                    "\t// n: 2\n"
                    "\t// End  1\n"
                    "\t// Code 1\n"
                    "\t// i: 1\n"
                    "\t// n: 2\n"
                    "\t// End  1\n"
                    "\t// Code 2\n"
                    "\t// i: 0\n"
                    "\t// n: 2\n"
                    "\t// End  2\n"
                    "\t// Code 2\n"
                    "\t// i: 1\n"
                    "\t// n: 2\n"
                    "\t// End  2\n"
                );
            }

            TEST(Loop_toString, multi_code_once)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/multi_code_once.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Not A Real Type";
                j["label"] = "Not A Real Label";
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr,
                    "\t// Code 1\n"
                    "\t// n: 2\n"
                    "\t// End  1\n"
                    "\t// Code 2\n"
                    "\t// n: 2\n"
                    "\t// End  2\n"
                );
            }

            TEST(Loop_toString, multi_code_mixed)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/multi_code_mixed.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Not A Real Type";
                j["label"] = "Not A Real Label";
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr,
                    "\t// Do something with 0\n"
                    "\t// Do something with 1\n"
                    "\t// Sleep once (50 * 2)\n"
                    "\t// Do something else with 0\n"
                    "\t// Do something else with 1\n"
                );
            }

            TEST(Loop_toString, code_insert_once_illegal)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/code_insert_once_illegal.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                nlohmann::json j;
                j["type"] = "Not A Real Type";
                j["label"] = "Not A Real Label";
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));
                appendages.emplace_back(std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                std::string loopStr;
                RIP_ASSERT_THROW(loopStr = loop->toString(appendages), IllegalPatternException);
            }
        }
    }
}
