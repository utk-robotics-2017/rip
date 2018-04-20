#include "arduino_gen/appendage.hpp"
#include "arduino_gen/exceptions.hpp"

#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

using rip::arduinogen::Appendage;
using rip::arduinogen::AppendageDataException;

namespace rip
{
    namespace arduinogen
    {
        namespace test
        {
            TEST(Appendage_type_check, Appendage_type)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["pin"] = 1;
                j["pullup"] = false;

                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);

                j["type"] = "";
                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_type_check, Appendage_label)
            {
                nlohmann::json j;
                j["type"] = "Digital Input";
                j["pin"] = 1;
                j["pullup"] = false;

                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);

                j["label"] = "";
                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_type_check, Appendage_missing_parameter)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                j["pin"] = 1;

                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_type_check, Appendage_extra_parameter)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                j["pin"] = 1;
                j["pullup"] = false;
                j["extra"] = 1;

                ASSERT_THROW(std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_missing_data)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                j["pin"] = 1;
                j["pullup"] = "";

                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_NO_THROW(appendage = std::make_shared<Appendage>(j));

                ASSERT_THROW(appendage->getString("extra"), AppendageDataException);

                ASSERT_THROW(appendage->isType("extra", "int"), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_unknown_type)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                j["pin"] = 1;
                j["pullup"] = "";

                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_NO_THROW(appendage = std::make_shared<Appendage>(j));

                ASSERT_THROW(appendage->isType("label", "unknown"), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_no_template)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Not A Type";
                j["pin"] = 1;
                j["pullup"] = false;

                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_THROW(appendage = std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_incorrect_bool)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Test";
                j["a_bool"] = "str";
                j["a_string"] = "str";
                j["an_int"] = 1;
                j["a_float"] = 4.0f;

                std::shared_ptr<Appendage> appendage;
                ASSERT_THROW(appendage = std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_incorrect_float)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Test";
                j["a_bool"] = true;
                j["a_string"] = "str";
                j["an_int"] = 1;
                j["a_float"] = "str";

                std::shared_ptr<Appendage> appendage;
                ASSERT_THROW(appendage = std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_incorrect_int)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Test";
                j["a_bool"] = true;
                j["a_string"] = "str";
                j["an_int"] = false;
                j["a_float"] = 1.0f;

                std::shared_ptr<Appendage> appendage;
                ASSERT_THROW(appendage = std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_data, Appendage_incorrect_string)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Test";
                j["a_bool"] = true;
                j["a_string"] = 1;
                j["an_int"] = 1;
                j["a_float"] = 3.0f;

                std::shared_ptr<Appendage> appendage;
                ASSERT_THROW(appendage = std::make_shared<Appendage>(j), AppendageDataException);
            }

            TEST(Appendage_type_check, DigitalInput)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "Digital Input";
                j["pin"] = 1;
                j["pullup"] = "";

                RIP_ASSERT_NO_THROW(std::make_shared<Appendage>(j));
            }

            TEST(Appendage_core_config, getCoreJson)
            {
                nlohmann::json j;
                j["label"] = "whatever";
                j["type"] = "something";
                j["pin"] = 1;

                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_NO_THROW(appendage = std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                ASSERT_EQ(appendage->getCoreJson(0).dump(4),
                    "{\n"
                    "    \"index\": 0,\n"
                    "    \"label\": \"whatever\",\n"
                    "    \"type\": \"something\"\n"
                    "}"
                );
            }

            TEST(Appendage_core_config, getCoreJson2)
            {
                nlohmann::json j;
                j["label"] = "something";
                j["type"] = "whatever";
                j["pin"] = 2;
                j["not"] = "used";

                std::shared_ptr<Appendage> appendage;
                RIP_ASSERT_NO_THROW(appendage = std::make_shared<Appendage>(j, std::vector<std::string>{}, false));

                ASSERT_EQ(appendage->getCoreJson(7).dump(4),
                    "{\n"
                    "    \"index\": 7,\n"
                    "    \"label\": \"something\",\n"
                    "    \"type\": \"whatever\"\n"
                    "}"
                );
            }
        }
    }
}
