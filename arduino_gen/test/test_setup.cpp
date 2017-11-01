#include "setup.hpp"
#include "exceptions.hpp"
#include "xml_utils.hpp"
#include "appendage.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

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
            TEST(Setup_constructor, no_attributes_no_elements)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/no_attributes_no_elements.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                ASSERT_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)), ElementException);
            }

            TEST(Setup_constructor, extra_attribute)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/extra_attribute.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                ASSERT_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)), ElementException);
            }

            TEST(Setup_constructor, extra_element)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/extra_element.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                ASSERT_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)), ElementException);
            }

            TEST(Setup_constructor, one_code_no_parameters)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/one_code_no_parameters.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));
            }

            TEST(Setup_constructor, two_codes_no_parameters)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/two_codes_no_parameters.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                ASSERT_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)), ElementException);
            }

            TEST(Setup_constructor, one_code_one_parameter_name_mismatch)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/one_code_one_parameter_name_mismatch.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::string setupStr;
                ASSERT_THROW(setupStr = setup->toString(appendages), PatternNotFoundException);
            }

            TEST(Setup_constructor, one_code_one_parameter_name_match)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/one_code_one_parameter_name_match.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::string setupStr;
                RIP_ASSERT_NO_THROW(setupStr = setup->toString(appendages));

                ASSERT_EQ(setupStr, "\tbravo\n");
            }

            TEST(Setup_constructor, one_code_two_parameter_name_match)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/setup/one_code_two_parameter_name_match.xml", {"code"}), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* setupElement = doc.FirstChildElement("setup");
                ASSERT_NE(setupElement, nullptr);

                std::unique_ptr<arduinogen::Setup> setup;
                RIP_ASSERT_NO_THROW(setup = std::unique_ptr<arduinogen::Setup>(new arduinogen::Setup(setupElement)));

                std::vector<std::shared_ptr<Appendage>> appendages;

                std::string setupStr;
                RIP_ASSERT_NO_THROW(setupStr = setup->toString(appendages));

                ASSERT_EQ(setupStr, "\tbravo\n"
                                    "\tdelta\n");
            }
        }
    }
}
