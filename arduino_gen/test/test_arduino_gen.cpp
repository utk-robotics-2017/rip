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

        TEST_F(ArduinoGenTest, setup_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1\n\n"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1\n\n"
                "\t// Ultrasonic triggerPin: 3\n\n"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\tservo.attach(3);\n\n"
                "\t// Ultrasonic triggerPin: 1\n\n"
            );
        }

        TEST_F(ArduinoGenTest, loop_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2\n\n"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2\n\n"
                "\t// Ultrasonic echoPin: 4\n\n"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Servo pin: 3\n\n"
                "\t// Ultrasonic echoPin: 2\n\n"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_enums_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_enums_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkSetServo,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(Commands::kReadUltrasonic, readUltraSonic);\n"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(Commands::kReadUltrasonic, readUltraSonic);\n"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(Commands::kSetServo, setServo);\n"
                "\tcmdMessenger.attach(Commands::kReadUltrasonic, readUltraSonic);\n"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[index_num].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(Commands::kAcknowledge, Commands::kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(Commands::kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendBinArg(rv);\n"
                "\tcmdMeessenger.sendCmdEnd();\n"
                "}\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 2) {\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[index_num].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(Commands::kAcknowledge, Commands::kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(Commands::kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendBinArg(rv);\n"
                "\tcmdMeessenger.sendCmdEnd();\n"
                "}\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                "void setServo() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tint value = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk()) {\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tif (!servos[indexNum].attached()) {\n"
                "\t    servos[indexNum].attach(servo_pins[indexNum]);\n"
                "\t}\n"
                "\tservos[indexNum].write(value);\n"
                "\tcmdMessenger.sendBinCmd(Commands::kAcknowledge, Commands::kSetServo);\n"
                "}\n"
                "\n"
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(Commands::kError, Commands::kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[index_num].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(Commands::kAcknowledge, Commands::kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(Commands::kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendBinArg(rv);\n"
                "\tcmdMeessenger.sendCmdEnd();\n"
                "}\n"
                "\n"
            );
        }
    }
}
