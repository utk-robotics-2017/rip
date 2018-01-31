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
                "Servo servos [1] = {\n"
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
                "\tservos[0].attach(3);\n\n"
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
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
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
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
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
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
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
                "\t\tcmdMessenger.sendBinCmd(kError, kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tint value = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk()) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tif (!servos[indexNum].attached()) {\n"
                "\t    servos[indexNum].attach(servo_pins[indexNum]);\n"
                "\t}\n"
                "\tservos[indexNum].write(value);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetServo);\n"
                "}\n"
                "\n"
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
                "}\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "\n"
                "\n"
                "enum\n"
                "{\n"
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\n"
                "};\n"
                "\n"
                "void setup()\n"
                "{\n"
                "\t// Init LED pin\n"
                "\tpinMode(LED, OUTPUT);\n"
                "\n"
                "\t// Initialize Serial Communication\n"
                "\tSerial.begin(115200);\n"
                "\n"
                "\tattachCommandCallbacks();\n"
                "\n"
                "\n"
                "\n"
                "\t// Flash led 3 times at the end of setup\n"
                "\tfor(int i = 0; i < 3; i++)\n"
                "\t{\n"
                "\t\tdigitalWrite(LED, HIGH);\n"
                "\t\tdelay(250);\n"
                "\t\tdigitalWrite(LED, LOW);\n"
                "\t\tdelay(250);\n"
                "\t}\n"
                "}\n"
                "\n"
                "void loop()\n"
                "{\n"
                "\t// Process incoming serial data, and perform callbacks\n"
                "\tcmdMessenger.feedinSerialData();\n"
                "\n"
                "\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\n"
                "}\n"
                "\n"
                "// Called when a received command has no attached function\n"
                "void unknownCommand()\n"
                "{\n"
                "\tcmdMessenger.sendCmd(kError, kUnknown);\n"
                "}\n"
                "\n"
                "// Called upon initialization of Spine to check connection\n"
                "void ping()\n"
                "{\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kPing);\n"
                "\tcmdMessenger.sendBinCmd(kPingResult, kPong);\n"
                "}\n"
                "\n"
                "// Callback function that sets led on or off\n"
                "void setLed()\n"
                "{\n"
                "\t// Read led state argument, interpret string as boolean\n"
                "\tbool ledState = cmdMessenger.readBoolArg();\n"
                "\tdigitalWrite(LED, ledState);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetLed);\n"
                "}\n"
                "\n"
                "\n"
                "\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "\n"
                "\n"
                "enum\n"
                "{\n"
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\n"
                "};\n"
                "\n"
                "void setup()\n"
                "{\n"
                "\t// Init LED pin\n"
                "\tpinMode(LED, OUTPUT);\n"
                "\n"
                "\t// Initialize Serial Communication\n"
                "\tSerial.begin(115200);\n"
                "\n"
                "\tattachCommandCallbacks();\n"
                "\n"
                "\n"
                "\n"
                "\t// Flash led 3 times at the end of setup\n"
                "\tfor(int i = 0; i < 3; i++)\n"
                "\t{\n"
                "\t\tdigitalWrite(LED, HIGH);\n"
                "\t\tdelay(250);\n"
                "\t\tdigitalWrite(LED, LOW);\n"
                "\t\tdelay(250);\n"
                "\t}\n"
                "}\n"
                "\n"
                "void loop()\n"
                "{\n"
                "\t// Process incoming serial data, and perform callbacks\n"
                "\tcmdMessenger.feedinSerialData();\n"
                "\n"
                "\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\n"
                "}\n"
                "\n"
                "// Called when a received command has no attached function\n"
                "void unknownCommand()\n"
                "{\n"
                "\tcmdMessenger.sendCmd(kError, kUnknown);\n"
                "}\n"
                "\n"
                "// Called upon initialization of Spine to check connection\n"
                "void ping()\n"
                "{\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kPing);\n"
                "\tcmdMessenger.sendBinCmd(kPingResult, kPong);\n"
                "}\n"
                "\n"
                "// Callback function that sets led on or off\n"
                "void setLed()\n"
                "{\n"
                "\t// Read led state argument, interpret string as boolean\n"
                "\tbool ledState = cmdMessenger.readBoolArg();\n"
                "\tdigitalWrite(LED, ledState);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetLed);\n"
                "}\n"
                "\n"
                "\n"
                "\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include <NewPing.h>\n\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n\n\n"
                "\n"
                "enum\n"
                "{\n"
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult\n"
                "};\n"
                "\n"
                "void setup()\n"
                "{\n"
                "\t// Init LED pin\n"
                "\tpinMode(LED, OUTPUT);\n"
                "\n"
                "\t// Initialize Serial Communication\n"
                "\tSerial.begin(115200);\n"
                "\n"
                "\tattachCommandCallbacks();\n"
                "\n"
                "\t// Ultrasonic triggerPin: 1\n\n\n"
                "\n"
                "\t// Flash led 3 times at the end of setup\n"
                "\tfor(int i = 0; i < 3; i++)\n"
                "\t{\n"
                "\t\tdigitalWrite(LED, HIGH);\n"
                "\t\tdelay(250);\n"
                "\t\tdigitalWrite(LED, LOW);\n"
                "\t\tdelay(250);\n"
                "\t}\n"
                "}\n"
                "\n"
                "void loop()\n"
                "{\n"
                "\t// Process incoming serial data, and perform callbacks\n"
                "\tcmdMessenger.feedinSerialData();\n"
                "\n"
                "\t// Ultrasonic echoPin: 2\n\n\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n\n"
                "}\n"
                "\n"
                "// Called when a received command has no attached function\n"
                "void unknownCommand()\n"
                "{\n"
                "\tcmdMessenger.sendCmd(kError, kUnknown);\n"
                "}\n"
                "\n"
                "// Called upon initialization of Spine to check connection\n"
                "void ping()\n"
                "{\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kPing);\n"
                "\tcmdMessenger.sendBinCmd(kPingResult, kPong);\n"
                "}\n"
                "\n"
                "// Callback function that sets led on or off\n"
                "void setLed()\n"
                "{\n"
                "\t// Read led state argument, interpret string as boolean\n"
                "\tbool ledState = cmdMessenger.readBoolArg();\n"
                "\tdigitalWrite(LED, ledState);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetLed);\n"
                "}\n"
                "\n"
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
                "}\n"
                "\n\n"
                "\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include <NewPing.h>\n\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "NewPing sonar [2] = {\n"
                "\tNewPing(1, 2, 200),\n"
                "\tNewPing(3, 4, 200)\n"
                "};\n\n\n"
                "\n"
                "enum\n"
                "{\n"
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult\n"
                "};\n"
                "\n"
                "void setup()\n"
                "{\n"
                "\t// Init LED pin\n"
                "\tpinMode(LED, OUTPUT);\n"
                "\n"
                "\t// Initialize Serial Communication\n"
                "\tSerial.begin(115200);\n"
                "\n"
                "\tattachCommandCallbacks();\n"
                "\n"
                "\t// Ultrasonic triggerPin: 1\n\n"
                "\t// Ultrasonic triggerPin: 3\n\n\n"
                "\n"
                "\t// Flash led 3 times at the end of setup\n"
                "\tfor(int i = 0; i < 3; i++)\n"
                "\t{\n"
                "\t\tdigitalWrite(LED, HIGH);\n"
                "\t\tdelay(250);\n"
                "\t\tdigitalWrite(LED, LOW);\n"
                "\t\tdelay(250);\n"
                "\t}\n"
                "}\n"
                "\n"
                "void loop()\n"
                "{\n"
                "\t// Process incoming serial data, and perform callbacks\n"
                "\tcmdMessenger.feedinSerialData();\n"
                "\n"
                "\t// Ultrasonic echoPin: 2\n\n"
                "\t// Ultrasonic echoPin: 4\n\n\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n\n"
                "}\n"
                "\n"
                "// Called when a received command has no attached function\n"
                "void unknownCommand()\n"
                "{\n"
                "\tcmdMessenger.sendCmd(kError, kUnknown);\n"
                "}\n"
                "\n"
                "// Called upon initialization of Spine to check connection\n"
                "void ping()\n"
                "{\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kPing);\n"
                "\tcmdMessenger.sendBinCmd(kPingResult, kPong);\n"
                "}\n"
                "\n"
                "// Callback function that sets led on or off\n"
                "void setLed()\n"
                "{\n"
                "\t// Read led state argument, interpret string as boolean\n"
                "\tbool ledState = cmdMessenger.readBoolArg();\n"
                "\tdigitalWrite(LED, ledState);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetLed);\n"
                "}\n"
                "\n"
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 2) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
                "}\n"
                "\n\n"
                "\n"
                "\n"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "/", "test/data/arduino_gen", true));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>\n\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "Servo servos [1] = {\n"
                "\tServo()\n"
                "};\n\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n\n\n"
                "\n"
                "enum\n"
                "{\n"
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkSetServo,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult\n"
                "};\n"
                "\n"
                "void setup()\n"
                "{\n"
                "\t// Init LED pin\n"
                "\tpinMode(LED, OUTPUT);\n"
                "\n"
                "\t// Initialize Serial Communication\n"
                "\tSerial.begin(115200);\n"
                "\n"
                "\tattachCommandCallbacks();\n"
                "\n"
                "\tservos[0].attach(3);\n\n"
                "\t// Ultrasonic triggerPin: 1\n\n\n"
                "\n"
                "\t// Flash led 3 times at the end of setup\n"
                "\tfor(int i = 0; i < 3; i++)\n"
                "\t{\n"
                "\t\tdigitalWrite(LED, HIGH);\n"
                "\t\tdelay(250);\n"
                "\t\tdigitalWrite(LED, LOW);\n"
                "\t\tdelay(250);\n"
                "\t}\n"
                "}\n"
                "\n"
                "void loop()\n"
                "{\n"
                "\t// Process incoming serial data, and perform callbacks\n"
                "\tcmdMessenger.feedinSerialData();\n"
                "\n"
                "\t// Servo pin: 3\n\n"
                "\t// Ultrasonic echoPin: 2\n\n\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n\n"
                "}\n"
                "\n"
                "// Called when a received command has no attached function\n"
                "void unknownCommand()\n"
                "{\n"
                "\tcmdMessenger.sendCmd(kError, kUnknown);\n"
                "}\n"
                "\n"
                "// Called upon initialization of Spine to check connection\n"
                "void ping()\n"
                "{\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kPing);\n"
                "\tcmdMessenger.sendBinCmd(kPingResult, kPong);\n"
                "}\n"
                "\n"
                "// Callback function that sets led on or off\n"
                "void setLed()\n"
                "{\n"
                "\t// Read led state argument, interpret string as boolean\n"
                "\tbool ledState = cmdMessenger.readBoolArg();\n"
                "\tdigitalWrite(LED, ledState);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetLed);\n"
                "}\n"
                "\n"
                "void setServo() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tint value = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk()) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kSetServo);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tif (!servos[indexNum].attached()) {\n"
                "\t    servos[indexNum].attach(servo_pins[indexNum]);\n"
                "\t}\n"
                "\tservos[indexNum].write(value);\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kSetServo);\n"
                "}\n"
                "\n"
                "void readUltraSonic() {\n"
                "\tint indexNum = cmdMessenger.readBinArg<int>();\n"
                "\tif(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {\n"
                "\t\tcmdMessenger.sendBinCmd(kError, kReadUltrasonic);\n"
                "\t\treturn;\n"
                "\t}\n"
                "\tunsigned long rv;\n"
                "\trv = sonar[indexNum].ping_cm();\n"
                "\tcmdMessenger.sendBinCmd(kAcknowledge, kReadUltrasonic);\n"
                "\tcmdMessenger.sendCmdStart(kReadUltrasonicResult);\n"
                "\tcmdMessenger.sendCmdBinArg(rv);\n"
                "\tcmdMessenger.sendCmdEnd();\n"
                "}\n"
                "\n\n"
                "\n"
                "\n"
            );
        }
    }
}
