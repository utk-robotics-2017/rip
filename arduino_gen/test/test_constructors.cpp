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
using rip::arduinogen::mmap_to_vector;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Constructors, NoTypeNoVariableConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/no_type_no_variable_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                ASSERT_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)), AttributeException);
            }

            TEST(Constructors, NoTypeConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/no_type_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                ASSERT_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)), AttributeException);
            }

            TEST(Constructors, NoVariableConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/no_variable_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                ASSERT_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)), AttributeException);
            }

            TEST(Constructors, EmptyConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/empty_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [0] = {\n"
                    "};\n"
                );
            }

            TEST(Constructors, IntArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/int_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = 1;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1)\n"
                    "};\n"
                );
            }

            TEST(Constructors, FloatArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/float_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = 3.0f;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(3.000000)\n"
                    "};\n"
                );
            }

            TEST(Constructors, BoolArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/bool_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(true)\n"
                    "};\n"
                );
            }

            TEST(Constructors, StringArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/string_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something"] = "asdf";

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 1u);

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(\"asdf\")\n"
                    "};\n"
                );
            }

            TEST(Constructors, SingleAppendageMultipleArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/multiple_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something_int"] = 1;
                j["something_float"] = 16.0f;
                j["something_string"] = "four";
                j["something_bool"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                appendage_map.emplace(std::make_pair("Other Type", std::make_shared<Appendage>(j, appendage_map, "", false)));
                appendage_map.emplace(std::make_pair("abc Type", std::make_shared<Appendage>(j, appendage_map, "", false)));
                appendage_map.emplace(std::make_pair("one more Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 1);

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1, true, 16.000000, \"four\")\n"
                    "};\n"
                );
            }

            TEST(Constructors, MultipleAppendageMultipleArgumentConstructor)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructors/multiple_argument_constructors.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructors> constructors;
                RIP_ASSERT_NO_THROW(constructors = std::unique_ptr<Constructors>(new Constructors(xml)));

                nlohmann::json j;
                j["something_int"] = 1;
                j["something_float"] = 16.0f;
                j["something_string"] = "four";
                j["something_bool"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                j["something_int"] = 2;
                j["something_float"] = 1290.0f;
                j["something_string"] = "three";
                j["something_bool"] = false;

                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                j["something_int"] = 3;
                j["something_float"] = 2.5f;
                j["something_string"] = "two";
                j["something_bool"] = true;

                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                j["something_int"] = 4;
                j["something_float"] = 1.125f;
                j["something_string"] = "one";
                j["something_bool"] = false;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                appendage_map.emplace(std::make_pair("Other Type", std::make_shared<Appendage>(j, appendage_map, "", false)));
                appendage_map.emplace(std::make_pair("abc Type", std::make_shared<Appendage>(j, appendage_map, "", false)));
                appendage_map.emplace(std::make_pair("one more Type", std::make_shared<Appendage>(j, appendage_map, "", false)));

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 4);

                ASSERT_EQ(constructors->toString(appendages),
                    "Type type [4] = {\n"
                    "\tType(1, true, 16.000000, \"four\"),\n"
                    "\tType(2, false, 1290.000000, \"three\"),\n"
                    "\tType(3, true, 2.500000, \"two\"),\n"
                    "\tType(4, false, 1.125000, \"one\")\n"
                    "};\n"
                );
            }
        }
    }
}
