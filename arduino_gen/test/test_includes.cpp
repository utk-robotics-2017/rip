#include "arduino_gen/includes.hpp"
#include "arduino_gen/exceptions.hpp"
#include "arduino_gen/xml_utils.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

using Includes = rip::arduinogen::Includes;
using AttributeException = rip::arduinogen::AttributeException;
using ElementException = rip::arduinogen::ElementException;
using rip::arduinogen::loadXmlFile;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Includes, EmptyInclude)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/includes/empty_includes.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* includesElement = doc.FirstChildElement("includes");
                ASSERT_NE(includesElement, nullptr);

                Includes includes(includesElement);
                std::vector<std::string> includes_vec = includes.GetIncludes();

                EXPECT_EQ(includes_vec.size(), 0u);
            }

            TEST(Includes, SingleInclude)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/includes/single_include.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* includesElement = doc.FirstChildElement("includes");
                ASSERT_NE(includesElement, nullptr);

                Includes includes(includesElement);
                std::vector<std::string> includes_vec = includes.GetIncludes();

                EXPECT_EQ(includes_vec.size(), 1);
                EXPECT_EQ(includes_vec[0], "\"Something.h\"");
            }

            TEST(Includes, MultipleIncludes)
            {
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(loadXmlFile(doc, "test/data/includes/multiple_includes.xml", {  }), tinyxml2::XML_SUCCESS);

                tinyxml2::XMLElement* includesElement = doc.FirstChildElement("includes");
                ASSERT_NE(includesElement, nullptr);

                Includes includes(includesElement);
                std::vector<std::string> includes_vec = includes.GetIncludes();

                EXPECT_EQ(includes_vec.size(), 2);
                EXPECT_EQ(includes_vec[0], "\"Something.h\"");
                EXPECT_EQ(includes_vec[1], "\"Somethingelse.h\"");
            }
        }
    }
}
