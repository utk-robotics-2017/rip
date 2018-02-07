#include "arduino_gen/setup.hpp"
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

//using Setup = rip::arduinogen::Setup;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Setup_constructor, no_attributes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/no_attributes.xml", {"code", "setup"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));
            }

            TEST(Setup_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/extra_attribute.xml", {"code", "setup"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                ASSERT_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)), AttributeException);
            }

            TEST(Setup_toString, analog_input_name_mismatch)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/analog_input_name_mismatch.xml", {"code", "setup"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string setupStr;
                ASSERT_THROW(setupStr = setup->toString(appendages), PatternNotFoundException);
            }

            TEST(Setup_toString, analog_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/analog_input.xml", {"code", "setup"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Analog Input";
                j["label"] = "Whatever";
                j["pin"] = 1;

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string setupStr;
                RIP_ASSERT_NO_THROW(setupStr = setup->toString(appendages));

                ASSERT_EQ(setupStr, "\t// 1\n");
            }

            TEST(Setup_toString, digital_input)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/digital_input.xml", {"code", "setup"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;
                std::multimap<std::string, std::shared_ptr<Appendage>> not_used;

                nlohmann::json j;
                j["type"] = "Digital Input";
                j["label"] = "Whatever";
                j["pin"] = 1;
                j["pullup"] = "_PULLUP";

                appendages.emplace_back(std::make_shared<Appendage>(j, not_used, "", false));

                std::string setupStr;
                RIP_ASSERT_NO_THROW(setupStr = setup->toString(appendages));

                ASSERT_EQ(setupStr, "\tpinMode(1, INPUT_PULLUP);\n");
            }
        }
    }
}
