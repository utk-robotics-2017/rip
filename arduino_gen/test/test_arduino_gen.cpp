#include "arduino_gen.hpp"
#include "appendage.hpp"
#include "appendage_template.hpp"
#include "exceptions.hpp"

#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <string>
#include <memory>

using ArduinoGen = rip::arduinogen::ArduinoGen;
using Appendage = rip::arduinogen::Appendage;

namespace rip
{
    namespace arduinogen
    {
        class ArduinoGenTest : public ::testing::Test {
        private:

        };

        TEST_F(ArduinoGenTest, includes_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            ag->loadTemplates();

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            nlohmann::json j;
            j["type"] = "Empty Appendage";
            j["label"] = "Empty1";

            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            ag->loadTemplates();

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            nlohmann::json j;
            j["type"] = "Ultrasonic";
            j["label"] = "Ultrasonic1";
            j["triggerPin"] = 1;
            j["echoPin"] = 2;

            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            ag->loadTemplates();

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>\n"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));
            nlohmann::json j;

            j["type"] = "Ultrasonic";
            j["label"] = "Ultrasonic1";
            j["triggerPin"] = 1;
            j["echoPin"] = 2;
            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            j["label"] = "Ultrasonic2";
            j["triggerPin"] = 3;
            j["echoPin"] = 4;
            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            ag->loadTemplates();

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>\n"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));
            nlohmann::json j;

            j["type"] = "Ultrasonic";
            j["label"] = "Ultrasonic1";
            j["triggerPin"] = 1;
            j["echoPin"] = 2;
            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            j["type"] = "Servo";
            j["label"] = "Servo1";
            j["pin"] = 3;
            ag->m_appendages.insert(std::make_pair(j["type"].get<std::string>(), std::make_shared<Appendage>(j, ag->m_appendages, false)));

            ag->loadTemplates();

            ASSERT_EQ(ag->getIncludes(),
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>\n"
            );
        }
    }
}
