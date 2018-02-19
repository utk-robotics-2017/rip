#include "arduino_gen/appendage.hpp"
#include "arduino_gen/command.hpp"
#include "arduino_gen/appendage_template.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/xml_utils.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Appendage = rip::arduinogen::Appendage;
using AppendageTemplate = rip::arduinogen::AppendageTemplate;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(AppendageTemplate_constructor, no_attributes_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/no_attributes_no_elements.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "no_attributes_no_elements", appendages)));

                EXPECT_EQ(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_EQ(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_EQ(appendageTemplate->GetSetup(), nullptr);
                EXPECT_EQ(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 0u);
            }

            TEST(AppendageTemplate_constructor, includes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/includes.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "includes", appendages)));

                EXPECT_NE(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_EQ(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_EQ(appendageTemplate->GetSetup(), nullptr);
                EXPECT_EQ(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 0);
            }

            TEST(AppendageTemplate_constructor, constructors)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/constructors.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "constructors", appendages)));

                EXPECT_EQ(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_NE(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_EQ(appendageTemplate->GetSetup(), nullptr);
                EXPECT_EQ(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 0);
            }

            TEST(AppendageTemplate_constructor, setup)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/setup.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "setup", appendages)));

                EXPECT_EQ(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_EQ(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_NE(appendageTemplate->GetSetup(), nullptr);
                EXPECT_EQ(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 0);
            }

            TEST(AppendageTemplate_constructor, loop)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/loop.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "loop", appendages)));

                EXPECT_EQ(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_EQ(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_EQ(appendageTemplate->GetSetup(), nullptr);
                EXPECT_NE(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 0);
            }

            TEST(AppendageTemplate_constructor, commands)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/commands.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "commands", appendages)));

                EXPECT_EQ(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_EQ(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_EQ(appendageTemplate->GetSetup(), nullptr);
                EXPECT_EQ(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 2);
            }

            TEST(AppendageTemplate_constructor, everything)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/everything.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                RIP_ASSERT_NO_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "everything", appendages)));

                EXPECT_NE(appendageTemplate->GetIncludes(), nullptr);
                EXPECT_NE(appendageTemplate->GetConstructors(), nullptr);
                EXPECT_NE(appendageTemplate->GetSetup(), nullptr);
                EXPECT_NE(appendageTemplate->GetLoop(), nullptr);
                EXPECT_EQ(appendageTemplate->GetCommands().size(), 3);
            }

            TEST(AppendageTemplate_constructor, two_includes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/two_includes.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                ASSERT_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "two_includes", appendages)), ElementException);
            }

            TEST(AppendageTemplate_constructor, two_constructors)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/two_constructors.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                ASSERT_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "two_constructors", appendages)), ElementException);
            }

            TEST(AppendageTemplate_constructor, two_setup)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/two_setup.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                ASSERT_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "two_setup", appendages)), ElementException);
            }

            TEST(AppendageTemplate_constructor, two_loop)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/two_loop.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                ASSERT_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "two_loop", appendages)), ElementException);
            }

            TEST(AppendageTemplate_constructor, two_commands)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/appendage_template/two_commands.xml", { "code", "setup", "loop" }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* appendageTemplateElement = doc.FirstChildElement("appendage-template");
                ASSERT_NE(appendageTemplateElement, nullptr);

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::unique_ptr<AppendageTemplate> appendageTemplate;
                ASSERT_THROW(appendageTemplate = std::unique_ptr<AppendageTemplate>(new AppendageTemplate(appendageTemplateElement, "two_commands", appendages)), ElementException);
            }
        }
    }
}
