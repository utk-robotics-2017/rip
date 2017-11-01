#include "loop.hpp"
#include "exceptions.hpp"
#include "xml_utils.hpp"
#include "appendage.hpp"

#include <tinyxml2.h>
#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <vector>
#include <memory>

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Loop_constructor, extra_attribute)
            {
                /*nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                std::multimap< std::string, std::shared_ptr<Appendage> > appendage_map;
                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_NO_THROW(appendage = std::make_share<Appendage>(j, appendage_map));*/
                std::unique_ptr<Loop> loop;
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/loop/extra_attribute.xml"), tinyxml2::XML_SUCCESS);
                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);
                ASSERT_THROW(loop=std::unique_ptr<Loop>(new Loop(loopElement)), AttributeException);
            }
            TEST(Loop_constructor, extra_element)
            {
                std::unique_ptr<Loop> loop;
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/loop/extra_attribute.xml"), tinyxml2::XML_SUCCESS);
                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);
                ASSERT_THROW(loop=std::unique_ptr<Loop>(new Loop(loopElement)), ElementException);
            }
            TEST(Loop_constructor, no_code)
            {
                std::unique_ptr<Loop> loop;
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/loop/no_code.xml"), tinyxml2::XML_SUCCESS);
                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);
                ASSERT_THROW(loop=std::unique_ptr<Loop>(new Loop(loopElement)), ElementException);
            }
            TEST(Loop_constructor, two_codes)
            {
                std::unique_ptr<Loop> loop;
                tinyxml2::XMLDocument doc;
                ASSERT_EQ(doc.LoadFile("test/data/loop/two_codes.xml"), tinyxml2::XML_SUCCESS);
                tinyxml2::XMLElement* loopElement = doc.FirstChildElement("loop");
                ASSERT_NE(loopElement, nullptr);
                ASSERT_THROW(loop=std::unique_ptr<Loop>(new Loop(loopElement)), ElementException);
            }
        }
    }
}
