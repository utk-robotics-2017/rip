#include "arduino_gen/arduino_gen.hpp"
#include "arduino_gen/appendage.hpp"
#include "arduino_gen/appendage_template.hpp"
#include "arduino_gen/exceptions.hpp"

#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <cppfs/cppfs.h>
#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getIncludes(),
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, constructors_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [2] = {\n"
                "\tNewPing(1, 2, 200),\n"
                "\tNewPing(3, 4, 200)\n"
                "};"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getConstructors(),
                "Servo servos [1] = {\n"
                "\tServo()\n"
                "};\n\n"
                "unsigned char servo_pins [1] = {\n"
                "\t3\n"
                "};\n\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};"
            );
        }

        TEST_F(ArduinoGenTest, setup_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1\n\n"
                "\t// Ultrasonic triggerPin: 3"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getSetup(),
                "\tservos[0].attach(3);\n\n"
                "\t// Ultrasonic triggerPin: 1"
            );
        }

        TEST_F(ArduinoGenTest, loop_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2\n\n"
                "\t// Ultrasonic echoPin: 4"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getLoop(),
                "\t// Servo pin: 3\n\n"
                "\t// Ultrasonic echoPin: 2"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandEnums(),
                "\tkAcknowledge,\n"
                "\tkError,\n"
                "\tkUnknown,\n"
                "\tkSetLed,\n"
                "\tkPing,\n"
                "\tkPingResult,\n"
                "\tkPong,\n"
                "\tkSetServo,\n"
                "\tkReadUltrasonic,\n"
                "\tkReadUltrasonicResult"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

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
                "}"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

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
                "}"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

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
                "}"
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

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
                "\tkPong\n"
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
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

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
                "\tkPong\n"
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
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include <NewPing.h>\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n"
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
                "\t// Ultrasonic triggerPin: 1\n"
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
                "\t// Ultrasonic echoPin: 2\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
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
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include <NewPing.h>\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "NewPing sonar [2] = {\n"
                "\tNewPing(1, 2, 200),\n"
                "\tNewPing(3, 4, 200)\n"
                "};\n"
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
                "\t// Ultrasonic triggerPin: 3\n"
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
                "\t// Ultrasonic echoPin: 4\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
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
            );
        }

        TEST_F(ArduinoGenTest, arduino_code_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "Servo servos [1] = {\n"
                "\tServo()\n"
                "};\n\n"
                "unsigned char servo_pins [1] = {\n"
                "\t3\n"
                "};\n\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n"
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
                "\t// Ultrasonic triggerPin: 1\n"
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
                "\t// Ultrasonic echoPin: 2\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
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
            );
        }

        TEST_F(ArduinoGenTest, get_core_config_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json", false));

            ASSERT_EQ(ag->getCoreConfig(),
                "{\n"
                "    \"appendages\": [],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );
        }

        TEST_F(ArduinoGenTest, get_core_config_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json", false));

            ASSERT_EQ(ag->getCoreConfig(),
                "{\n"
                "    \"appendages\": [\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Empty1\",\n"
                "            \"type\": \"Empty Appendage\"\n"
                "        }\n"
                "    ],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );
        }

        TEST_F(ArduinoGenTest, get_core_config_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json", false));

            ASSERT_EQ(ag->getCoreConfig(),
                "{\n"
                "    \"appendages\": [\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Ultrasonic1\",\n"
                "            \"type\": \"Ultrasonic\"\n"
                "        }\n"
                "    ],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kReadUltrasonic\": 7,\n"
                "        \"kReadUltrasonicResult\": 8,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );
        }

        TEST_F(ArduinoGenTest, get_core_config_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json", false));

            ASSERT_EQ(ag->getCoreConfig(),
                "{\n"
                "    \"appendages\": [\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Ultrasonic1\",\n"
                "            \"type\": \"Ultrasonic\"\n"
                "        },\n"
                "        {\n"
                "            \"index\": 1,\n"
                "            \"label\": \"Ultrasonic2\",\n"
                "            \"type\": \"Ultrasonic\"\n"
                "        }\n"
                "    ],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kReadUltrasonic\": 7,\n"
                "        \"kReadUltrasonicResult\": 8,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );
        }

        TEST_F(ArduinoGenTest, get_core_config_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", false));

            ASSERT_EQ(ag->getCoreConfig(),
                "{\n"
                "    \"appendages\": [\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Servo1\",\n"
                "            \"type\": \"Servo\"\n"
                "        },\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Ultrasonic1\",\n"
                "            \"type\": \"Ultrasonic\"\n"
                "        }\n"
                "    ],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kReadUltrasonic\": 8,\n"
                "        \"kReadUltrasonicResult\": 9,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kSetServo\": 7,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );
        }

        TEST_F(ArduinoGenTest, get_upload_script)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            ASSERT_EQ(ag->getUploadScript(),
                "#!/usr/bin/env bash\n"
                "\n"
                "cd ./test/CurrentArduinoCode/mega\n"
                "\n"
                "git add -A\n"
                "git commit -m \"New code for mega\"\n"
                "git push\n"
                "\n"
                "pio run -t upload\n"
            );
        }

        TEST_F(ArduinoGenTest, setup_folder)
        {
            using namespace cppfs;

            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            FileHandle device_folder = fs::open("./test/mega");

            // Create Blank slate
            if (device_folder.exists())
            {
                device_folder.removeDirectoryRec();
            }

            // Create directory with an empty file to make sure setupFolder erases it.
            device_folder.createDirectory();
            FileHandle test_file = fs::open("./test/mega/erase_me");
            test_file.writeFile("");

            RIP_ASSERT_NO_THROW(ag->setupFolder());

            device_folder.updateFileInfo();
            ASSERT_TRUE(device_folder.exists());
            EXPECT_TRUE(device_folder.isDirectory());

            FileHandle src_folder = fs::open("./test/mega/src");
            ASSERT_TRUE(src_folder.exists());
            EXPECT_TRUE(src_folder.isDirectory());

            #ifndef _WIN32
            EXPECT_EQ(device_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                                   FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                                   FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(src_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                                FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                                FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            #endif

            // Make sure the test_file was erased during setupFolder
            test_file.updateFileInfo();
            ASSERT_FALSE(test_file.exists());

            // Cleanup
            device_folder.removeDirectoryRec();
        }

        TEST_F(ArduinoGenTest, generate_output)
        {
            using namespace cppfs;

            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", "test/data/arduino_gen"));

            RIP_ASSERT_NO_THROW(ag->setupFolder());
            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json", true));
            RIP_ASSERT_NO_THROW(ag->generateOutput());

            FileHandle source = fs::open("./test/mega/src/mega.ino");
            ASSERT_TRUE(source.exists());
            EXPECT_TRUE(source.isFile());
            EXPECT_EQ(source.readFile(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#include \"CmdMessenger.h\"\n"
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>\n"
                "\n"
                "// Attach a new CmdMessenger object to the default Serial port\n"
                "CmdMessenger cmdMessenger = CmdMessenger(Serial);\n"
                "\n"
                "const char LED = 13;\n"
                "\n"
                "Servo servos [1] = {\n"
                "\tServo()\n"
                "};\n\n"
                "unsigned char servo_pins [1] = {\n"
                "\t3\n"
                "};\n\n"
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};\n"
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
                "\t// Ultrasonic triggerPin: 1\n"
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
                "\t// Ultrasonic echoPin: 2\n"
                "}\n"
                "\n"
                "//Callbacks define on which received commands we take action\n"
                "void attachCommandCallbacks()\n"
                "{\n"
                "\tcmdMessenger.attach(unknownCommand);\n"
                "\tcmdMessenger.attach(kPing, ping);\n"
                "\tcmdMessenger.attach(kSetLed, setLed);\n"
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);\n"
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
            );

            FileHandle config = fs::open("./test/mega/core_config.json");
            ASSERT_TRUE(config.exists());
            EXPECT_TRUE(config.isFile());
            EXPECT_EQ(config.readFile(),
                "{\n"
                "    \"appendages\": [\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Servo1\",\n"
                "            \"type\": \"Servo\"\n"
                "        },\n"
                "        {\n"
                "            \"index\": 0,\n"
                "            \"label\": \"Ultrasonic1\",\n"
                "            \"type\": \"Ultrasonic\"\n"
                "        }\n"
                "    ],\n"
                "    \"commands\": {\n"
                "        \"kAcknowledge\": 0,\n"
                "        \"kError\": 1,\n"
                "        \"kPing\": 4,\n"
                "        \"kPingResult\": 5,\n"
                "        \"kPong\": 6,\n"
                "        \"kReadUltrasonic\": 8,\n"
                "        \"kReadUltrasonicResult\": 9,\n"
                "        \"kSetLed\": 3,\n"
                "        \"kSetServo\": 7,\n"
                "        \"kUnknown\": 2\n"
                "    }\n"
                "}"
            );

            FileHandle upload = fs::open("./test/mega/upload.sh");
            ASSERT_TRUE(upload.exists());
            EXPECT_TRUE(upload.isFile());
            EXPECT_EQ(upload.readFile(),
                "#!/usr/bin/env bash\n"
                "\n"
                "cd ./test/CurrentArduinoCode/mega\n"
                "\n"
                "git add -A\n"
                "git commit -m \"New code for mega\"\n"
                "git push\n"
                "\n"
                "pio run -t upload\n"
            );

            #ifndef _WIN32
            EXPECT_EQ(source.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead | FilePermissions::OtherWrite);
            EXPECT_EQ(config.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead | FilePermissions::OtherWrite);
            EXPECT_EQ(upload.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                            FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            #endif

            // Cleanup
            fs::open("./test/mega").removeDirectoryRec();
        }
    }
}
