#include "template_parser.hpp"
#include "command.hpp"
#include "appendage.hpp"
#include "constructors.hpp"
#include "argument.hpp"
#include "exceptions.hpp"

#include <tinyxml2.h>
#include <json.hpp>
#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <memory>

using AttributeException = arduinogen::AttributeException;
using Appendage = arduinogen::Appendage;
using Constructors = arduinogen::Constructors;

namespace arduinogentests
{
    TEST(TemplateParser_general, EmptyFile)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/empty.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* includes = doc.FirstChildElement("includes");

        ASSERT_EQ(includes, nullptr);
    }

    TEST(TemplateParser_parseIncludes, EmptyInclude)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_includes/empty_includes.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* includes = doc.FirstChildElement("includes");

        ASSERT_NE(includes, nullptr);

        std::vector<std::string> includes_vec = arduinogen::templateparserdetail::parseIncludes(includes);

        ASSERT_EQ(includes_vec.size(), 0);
    }

    TEST(TemplateParser_parseIncludes, SingleInclude)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_includes/single_include.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* includes = doc.FirstChildElement("includes");

        ASSERT_NE(includes, nullptr);

        std::vector<std::string> includes_vec = arduinogen::templateparserdetail::parseIncludes(includes);

        ASSERT_EQ(includes_vec.size(), 1);

        ASSERT_EQ(includes_vec[0], "Something.h");
    }

    TEST(TemplateParser_parseIncludes, MultipleIncludes)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_includes/multiple_includes.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* includes = doc.FirstChildElement("includes");

        ASSERT_NE(includes, nullptr);

        std::vector<std::string> includes_vec = arduinogen::templateparserdetail::parseIncludes(includes);

        ASSERT_EQ(includes_vec.size(), 2);

        ASSERT_EQ(includes_vec[0], "Something.h");
        ASSERT_EQ(includes_vec[1], "Somethingelse.h");
    }

    TEST(TemplateParserTest_parseConstructors, EmptyConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/empty_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_EQ(xml, nullptr);

        Constructors constructors(xml);

        std::vector< std::shared_ptr<Appendage> > appendages;

        ASSERT_EQ(constructors.toString(appendages), "");
    }

    TEST(TemplateParserTest_parseConstructors, NoTypeConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/no_type_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        ASSERT_THROW(Constructors constructors(xml), AttributeException);
    }

    TEST(TemplateParserTest_parseConstructors, NoVariableConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/no_variable_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        ASSERT_THROW(Constructors constructors(xml), AttributeException);
    }

    TEST(TemplateParserTest_parseConstructors, IntArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/int_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something"] = 1;

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(1)\n};\n");
    }

    TEST(TemplateParserTest_parseConstructors, FloatArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/float_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something"] = 3.0f;

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(3.000000)\n};\n");
    }

    TEST(TemplateParserTest_parseConstructors, BoolArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/bool_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);
       tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something"] = true;

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(true)\n};\n");
    }

    TEST(TemplateParserTest_parseConstructors, StringArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/string_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something"] = "asdf";

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });
        ASSERT_EQ(appendages.size(), 1);

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(\"asdf\")\n};\n");
    }

    TEST(TemplateParserTest_parseConstructors, SingleAppendageMultipleArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/multiple_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something_int"] = 1;
        j["something_float"] = 16.0f;
        j["something_string"] = "four";
        j["something_bool"] = true;

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));


        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("Other Type", std::make_shared<Appendage>(j, appendage_map, false)));
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("abc Type", std::make_shared<Appendage>(j, appendage_map, false)));
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("one more Type", std::make_shared<Appendage>(j, appendage_map, false)));


        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });
        ASSERT_EQ(appendages.size(), 1);

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(1, true, 16.000000, \"four\")\n};\n");
    }

    TEST(TemplateParserTest_parseConstructors, MultipleAppendageMultipleArgumentConstructor)
    {
        tinyxml2::XMLDocument doc;
        ASSERT_NE(doc.LoadFile("test/data/template_parser/parse_constructors/multiple_argument_constructors.xml"), tinyxml2::XML_ERROR_FILE_NOT_FOUND);

        tinyxml2::XMLElement* xml = doc.FirstChildElement("constructors");

        ASSERT_NE(xml, nullptr);

        Constructors constructors(xml);

        nlohmann::json j;
        j["something_int"] = 1;
        j["something_float"] = 16.0f;
        j["something_string"] = "four";
        j["something_bool"] = true;

        std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        j["something_int"] = 2;
        j["something_float"] = 1290.0f;
        j["something_string"] = "three";
        j["something_bool"] = false;

        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        j["something_int"] = 3;
        j["something_float"] = 2.5f;
        j["something_string"] = "two";
        j["something_bool"] = true;

        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        j["something_int"] = 4;
        j["something_float"] = 1.125f;
        j["something_string"] = "one";
        j["something_bool"] = false;
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("A Type", std::make_shared<Appendage>(j, appendage_map, false)));

        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("Other Type", std::make_shared<Appendage>(j, appendage_map, false)));
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("abc Type", std::make_shared<Appendage>(j, appendage_map, false)));
        appendage_map.emplace(std::make_pair<std::string, std::shared_ptr<Appendage> >("one more Type", std::make_shared<Appendage>(j, appendage_map, false)));


        std::vector< std::shared_ptr<Appendage> > appendages;
        std::transform(appendage_map.lower_bound("A Type"),
                       appendage_map.upper_bound("A Type"),
                       std::back_inserter(appendages),
                       [](std::pair<std::string, std::shared_ptr<Appendage>> element)
        {
            return element.second;
        });
        ASSERT_EQ(appendages.size(), 4);

        ASSERT_EQ(constructors.toString(appendages), "Type type = {\n\tType(1, true, 16.000000, \"four\"),\n\tType(2, false, 1290.000000, \"three\"),\n\tType(3, true, 2.500000, \"two\"),\n\tType(4, false, 1.125000, \"one\")\n};\n");
    }

#ifdef not_defined

    TEST(TemplateParserTest, SetupParsing)
    {
        "setup_test.xml"
    }

    TEST(TemplateParserTest, LoopParsing)
    {
        "loop_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "no_parameter_no_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "single_parameter_no_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "no_parameter_single_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "single_parameter_single_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "multiple_parameter_no_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "no_parameter_multiple_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, NoParameterNoReturnValueCommandParsing)
    {
        "multiple_parameter_multiple_return_value_command_test.xml"
    }

    TEST(TemplateParserTest, ExtraParsing)
    {
        "extra_test.xml"
    }

    TEST(TemplateParserTest, CoreConfigParsing)
    {
        "core_config_test.xml"
    }
#endif
}
