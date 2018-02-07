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
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/no_attributes.xml", {"code", "loop"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));
            }

            TEST(Loop_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/extra_attribute.xml", {"code", "loop"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                ASSERT_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)), AttributeException);
            }

            TEST(Loop_toString, analog_input_name_mismatch)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/analog_input_name_mismatch.xml", {"code", "loop"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string loopStr;
                ASSERT_THROW(loopStr = loop->toString(appendages), PatternNotFoundException);
            }

            TEST(Loop_toString, analog_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/analog_input.xml", {"code", "loop"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr, "\tval = analogRead(1);\n");
            }

            TEST(Loop_toString, digital_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/loop/digital_input.xml", {"code", "loop"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);

                std::unique_ptr<Loop> loop;
                RIP_ASSERT_NO_THROW(loop = std::unique_ptr<Loop>(new Loop(loopElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Digital Input";
                j["label"] = "Whatever";
                j["pin"] = 1;
                j["pullup"] = "_PULLUP";

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string loopStr;
                RIP_ASSERT_NO_THROW(loopStr = loop->toString(appendages));

                ASSERT_EQ(loopStr, "\tval = digitalRead(1);\n");
            }
        }
    }
}
