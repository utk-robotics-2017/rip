#include "arduino_gen/constructors.hpp"
#include "arduino_gen/appendage.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/xml_utils.hpp"
#include "arduino_gen/utils.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Constructors = rip::arduinogen::Constructors;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;
using rip::arduinogen::get_mmap_values_at_index;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Constructors, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/extra_attribute.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                ASSERT_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)), AttributeException);
            }

            TEST(Constructors, extra_element)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/extra_element.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                ASSERT_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)), ElementException);
            }

            TEST(Constructors, zero_constructors)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/zero_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = 1;
                j["somethingElse"] = false;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    ""
                );
            }

            TEST(Constructors, one_constructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/one_constructor.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = 1;
                j["somethingElse"] = false;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1)\n"
                    "};\n"
                );
            }

            TEST(Constructors, two_constructors)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/two_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = 1;
                j["somethingElse"] = false;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    "IntType intType [1] = {\n"
                    "\tIntType(1)\n"
                    "};\n"
                    "\n"
                    "BoolType boolType [1] = {\n"
                    "\tBoolType(false)\n"
                    "};\n"
                );
            }
        }
    }
}
