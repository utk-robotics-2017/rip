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

        // TODO: Test readConfig

        TEST_F(ArduinoGenTest, includes_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>\n"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>\n"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>\n"
            );
        }

        TEST_F(ArduinoGenTest, constructors_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n\n"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [2] = {\n"
                "\tNewPing(1, 2, 200),\n"
                "\tNewPing(3, 4, 200)\n"
                "};\n\n"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "Servo servo [1] = {\n"
                "\tServo()\n"
                "};\n\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n\n"
            );
        }
    }
}
