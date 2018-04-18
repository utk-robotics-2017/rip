#include "arduino_gen/constructor.hpp"
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

using Constructor = rip::arduinogen::Constructor;
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
            TEST(Constructor, NoTypeNoVariable)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/no_type_no_variable.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                ASSERT_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)), AttributeException);
            }

            TEST(Constructor, NoType)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/no_type.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                ASSERT_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)), AttributeException);
            }

            TEST(Constructor, NoVariable)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/no_variable.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                ASSERT_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)), AttributeException);
            }

            TEST(Constructor, Empty)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/empty.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [0] = {\n"
                    "};\n"
                );
            }

            TEST(Constructor, IntArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/int_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something"] = 1;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1)\n"
                    "};\n"
                );
            }

            TEST(Constructor, FloatArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/float_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something"] = 3.0f;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(3.000000)\n"
                    "};\n"
                );
            }

            TEST(Constructor, BoolArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/bool_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(true)\n"
                    "};\n"
                );
            }

            TEST(Constructor, StringArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/string_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something"] = "asdf";

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 1);

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(\"asdf\")\n"
                    "};\n"
                );
            }

            TEST(Constructor, SingleAppendageMultipleArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/multiple_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something_int"] = 1;
                j["something_float"] = 16.0f;
                j["something_string"] = "four";
                j["something_bool"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                appendage_map.emplace(std::make_pair("Other Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));
                appendage_map.emplace(std::make_pair("abc Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));
                appendage_map.emplace(std::make_pair("one more Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 1);

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1, true, 16.000000, \"four\")\n"
                    "};\n"
                );
            }

            TEST(Constructor, MultipleAppendageMultipleArgument)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/multiple_argument.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["something_int"] = 1;
                j["something_float"] = 16.0f;
                j["something_string"] = "four";
                j["something_bool"] = true;

                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                j["something_int"] = 2;
                j["something_float"] = 1290.0f;
                j["something_string"] = "three";
                j["something_bool"] = false;

                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                j["something_int"] = 3;
                j["something_float"] = 2.5f;
                j["something_string"] = "two";
                j["something_bool"] = true;

                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                j["something_int"] = 4;
                j["something_float"] = 1.125f;
                j["something_string"] = "one";
                j["something_bool"] = false;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                appendage_map.emplace(std::make_pair("Other Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));
                appendage_map.emplace(std::make_pair("abc Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));
                appendage_map.emplace(std::make_pair("one more Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(appendages.size(), 4);

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [4] = {\n"
                    "\tType(1, true, 16.000000, \"four\"),\n"
                    "\tType(2, false, 1290.000000, \"three\"),\n"
                    "\tType(3, true, 2.500000, \"two\"),\n"
                    "\tType(4, false, 1.125000, \"one\")\n"
                    "};\n"
                );
            }

            TEST(Constructor, type_is_class_true)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/type_is_class_true.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["pin"] = 1;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructor->toString(appendages),
                    "Type type [1] = {\n"
                    "\tType(1)\n"
                    "};\n"
                );
            }

            TEST(Constructor, type_is_class_false)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/constructor/type_is_class_false.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* xml = doc.FirstChildElement("constructor");
                ASSERT_NE(xml, nullptr);

                std::unique_ptr<Constructor> constructor;
                RIP_ASSERT_NO_THROW(constructor = std::unique_ptr<Constructor>(new Constructor(xml)));

                nlohmann::json j;
                j["pin"] = 1;

                std::multimap<std::string, std::shared_ptr<Appendage>> appendage_map;
                appendage_map.emplace(std::make_pair("A Type", std::make_shared<Appendage>(j, appendage_map, std::vector<std::string>{}, false)));

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(appendage_map, "A Type");

                ASSERT_EQ(constructor->toString(appendages),
                    "unsigned char pins [1] = {\n"
                    "\t1\n"
                    "};\n"
                );
            }
        }
    }
}
