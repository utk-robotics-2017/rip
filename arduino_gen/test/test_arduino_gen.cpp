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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getIncludes(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, includes_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getIncludes(),
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, includes_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

            ASSERT_EQ(ag->getIncludes(),
                "#include \"Servo.h\"\n"
                "#include <NewPing.h>"
            );
        }

        TEST_F(ArduinoGenTest, constructors_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getConstructors(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, constructors_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [1] = {\n"
                "\tNewPing(1, 2, 200)\n"
                "};"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getConstructors(),
                "NewPing sonar [2] = {\n"
                "\tNewPing(1, 2, 200),\n"
                "\tNewPing(3, 4, 200)\n"
                "};"
            );
        }

        TEST_F(ArduinoGenTest, constructors_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getSetup(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, setup_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getSetup(),
                "\t// Ultrasonic triggerPin: 1\n\n"
                "\t// Ultrasonic triggerPin: 3"
            );
        }

        TEST_F(ArduinoGenTest, setup_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

            ASSERT_EQ(ag->getSetup(),
                "\tservos[0].attach(3);\n\n"
                "\t// Ultrasonic triggerPin: 1"
            );
        }

        TEST_F(ArduinoGenTest, loop_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getLoop(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, loop_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getLoop(),
                "\t// Ultrasonic echoPin: 2\n\n"
                "\t// Ultrasonic echoPin: 4"
            );
        }

        TEST_F(ArduinoGenTest, loop_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

            ASSERT_EQ(ag->getLoop(),
                "\t// Servo pin: 3\n\n"
                "\t// Ultrasonic echoPin: 2"
            );
        }

        TEST_F(ArduinoGenTest, command_enums_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getCommandAttaches(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_same)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_attaches_two_appendages_different)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

            ASSERT_EQ(ag->getCommandAttaches(),
                "\tcmdMessenger.attach(kSetServo, setServo);\n"
                "\tcmdMessenger.attach(kReadUltrasonic, readUltraSonic);"
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_no_appendages)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_empty_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getCommandCallbacks(),
                ""
            );
        }

        TEST_F(ArduinoGenTest, command_callbacks_one_appendage)
        {
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 7\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 7\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 9\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 9\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

            ASSERT_EQ(ag->getArduinoCode(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 10\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/no_appendages.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_empty_appendage.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/one_appendage.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_same.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));

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
            std::unique_ptr<ArduinoGen> ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" }));

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

        TEST_F(ArduinoGenTest, generate_output)
        {
            using namespace cppfs;

            //Create cppfs file handles
            FileHandle device_folder = fs::open("./test/mega");
            FileHandle source_folder = fs::open("./test/mega/src");

            FileHandle source = fs::open("./test/mega/src/mega.ino");
            FileHandle config = fs::open("./test/mega/mega.json");
            FileHandle core = fs::open("./test/mega/mega_core.json");
            FileHandle upload = fs::open("./test/mega/upload.sh");
            FileHandle serial = fs::open("./test/mega/serial.sh");
            FileHandle platformio = fs::open("./test/mega/platformio.ini");

            FileHandle erase_me = fs::open("./test/mega/erase_me");

            // Create Blank slate
            if (device_folder.exists())
            {
                device_folder.removeDirectoryRec();
            }

            // Create directory with an empty file to make sure setupFolder erases it.
            device_folder.createDirectory();
            FileHandle test_file = fs::open("./test/mega/erase_me");
            test_file.writeFile("");

            // Construct, readConfig, and generateOutput
            std::unique_ptr<ArduinoGen> ag;
            RIP_ASSERT_NO_THROW(ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" })));
            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));
            RIP_ASSERT_NO_THROW(ag->generateOutput(false));

            // Update file handles
            device_folder.updateFileInfo();
            source_folder.updateFileInfo();
            source.updateFileInfo();
            config.updateFileInfo();
            core.updateFileInfo();
            upload.updateFileInfo();
            serial.updateFileInfo();
            platformio.updateFileInfo();
            erase_me.updateFileInfo();

            // Test device folder
            ASSERT_TRUE(device_folder.exists());
            ASSERT_TRUE(device_folder.isDirectory());

            // Get all files in the device folder
            std::vector<std::string> files = device_folder.listFiles();

            // Test source folder and remove the name from the files vector
            ASSERT_TRUE(source_folder.exists());
            ASSERT_TRUE(source_folder.isDirectory());
            files.erase(std::find(files.begin(), files.end(), "src"));

            // Get all files in the source folder, prepend "src/" to the source folder files, and
            // append the source folder files to the files vector
            std::vector<std::string> source_folder_files = source_folder.listFiles();
            std::for_each(source_folder_files.begin(), source_folder_files.end(), [](std::string &str){ str = "src/" + str; });
            files.insert(files.end(), source_folder_files.begin(), source_folder_files.end());

            // Make sure the test file doesn't exist
            ASSERT_FALSE(test_file.exists());

            // Test src/mega.ino and remove it from the files vector
            ASSERT_TRUE(source.exists());
            ASSERT_TRUE(source.isFile());
            EXPECT_EQ(source.readFile(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 10\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            files.erase(std::find(files.begin(), files.end(), "src/mega.ino"));

            // Test mega.json and remove it from the files vector
            ASSERT_TRUE(config.exists());
            ASSERT_TRUE(config.isFile());
            EXPECT_EQ(config.readFile(),
                "[\n"
                "    {\n"
                "        \"type\": \"Ultrasonic\",\n"
                "        \"label\": \"Ultrasonic1\",\n"
                "        \"triggerPin\": 1,\n"
                "        \"echoPin\": 2\n"
                "    },\n"
                "    {\n"
                "        \"type\": \"Servo\",\n"
                "        \"label\": \"Servo1\",\n"
                "        \"pin\": 3\n"
                "    }\n"
                "]\n"
            );
            files.erase(std::find(files.begin(), files.end(), "mega.json"));

            // Test mega_core.json and remove it from the files vector
            ASSERT_TRUE(core.exists());
            ASSERT_TRUE(core.isFile());
            EXPECT_EQ(core.readFile(),
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
            files.erase(std::find(files.begin(), files.end(), "mega_core.json"));

            // Test upload.sh and remove it from the files vector
            ASSERT_TRUE(upload.exists());
            ASSERT_TRUE(upload.isFile());
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
            files.erase(std::find(files.begin(), files.end(), "upload.sh"));

            // Test serial.sh and remove it from the files vector
            ASSERT_TRUE(serial.exists());
            ASSERT_TRUE(serial.isFile());
            EXPECT_EQ(serial.readFile(),
                "#!/usr/bin/env bash\n"
                "\n"
                "picocom /dev/mega -b 115200 --echo\n"
            );
            files.erase(std::find(files.begin(), files.end(), "serial.sh"));

            // Test platformio.ini and remove it from the files vector
            ASSERT_TRUE(platformio.exists());
            ASSERT_TRUE(platformio.isFile());
            EXPECT_EQ(platformio.readFile(),
                "[platformio]\n"
                "lib_dir = /Robot/ArduinoLibraries\n"
                "env_default = mega\n"
                "\n"
                "[env:mega]\n"
                "platform = atmelavr\n"
                "framework = arduino\n"
                "board = uno\n"
                "upload_port = /dev/mega\n"
            );
            files.erase(std::find(files.begin(), files.end(), "platformio.ini"));

            // Make sure there aren't any extra files
            ASSERT_EQ(files.size(), 0);

            // Check the file permissions
            #ifndef _WIN32
            // TODO setGID support
            // EXPECT_EQ(device_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                                        FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                                        FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            // EXPECT_EQ(source_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                                        FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                                        FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(source.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(config.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(core.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                          FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                          FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(upload.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(serial.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(platformio.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                                FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                                FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            #endif

            // Change the platformio.ini file
            platformio.writeFile(
                "[platformio]\n"
                "lib_dir = /Robot/ArduinoLibraries\n"
                "env_default = mega\n"
                "\n"
                "[env:mega]\n"
                "platform = atmelavr\n"
                "framework = arduino\n"
                "board = mega\n"                      // from "uno" to "mega"
                "upload_port = /dev/mega\n"
            );

            // Construct, readConfig, and generateOutput
            RIP_ASSERT_NO_THROW(ag = std::unique_ptr<ArduinoGen>(new ArduinoGen("mega", "./test", "./test/CurrentArduinoCode", { "test/data/arduino_gen" })));
            RIP_ASSERT_NO_THROW(ag->readConfig("test/data/arduino_gen/two_appendages_different.json"));
            RIP_ASSERT_NO_THROW(ag->generateOutput(true));

            // Update file handles
            device_folder.updateFileInfo();
            source_folder.updateFileInfo();
            source.updateFileInfo();
            config.updateFileInfo();
            core.updateFileInfo();
            upload.updateFileInfo();
            serial.updateFileInfo();
            platformio.updateFileInfo();

            // Test device folder
            ASSERT_TRUE(device_folder.exists());
            ASSERT_TRUE(device_folder.isDirectory());

            // Get all files in the device folder
            files = device_folder.listFiles();

            // Test source folder and remove the name from the files vector
            ASSERT_TRUE(source_folder.exists());
            ASSERT_TRUE(source_folder.isDirectory());
            files.erase(std::find(files.begin(), files.end(), "src"));

            // Get all files in the source folder, prepend "src/" to the source folder files, and
            // append the source folder files to the files vector
            source_folder_files = source_folder.listFiles();
            std::for_each(source_folder_files.begin(), source_folder_files.end(), [](std::string &str){ str = "src/" + str; });
            files.insert(files.end(), source_folder_files.begin(), source_folder_files.end());

            // Test src/mega.ino and remove it from the files vector
            ASSERT_TRUE(source.exists());
            ASSERT_TRUE(source.isFile());
            EXPECT_EQ(source.readFile(),
                "// Auto-generated by ArduinoGen\n"
                "\n"
                "#define MAXCALLBACKS 10\n"
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
                "enum RIPenum : int16_t\n"
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
                "\tcmdMessenger.sendBinCmd(kError, kUnknown);\n"
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
            files.erase(std::find(files.begin(), files.end(), "src/mega.ino"));

            // Test mega.json and remove it from the files vector
            ASSERT_TRUE(config.exists());
            ASSERT_TRUE(config.isFile());
            EXPECT_EQ(config.readFile(),
                "[\n"
                "    {\n"
                "        \"type\": \"Ultrasonic\",\n"
                "        \"label\": \"Ultrasonic1\",\n"
                "        \"triggerPin\": 1,\n"
                "        \"echoPin\": 2\n"
                "    },\n"
                "    {\n"
                "        \"type\": \"Servo\",\n"
                "        \"label\": \"Servo1\",\n"
                "        \"pin\": 3\n"
                "    }\n"
                "]\n"
            );
            files.erase(std::find(files.begin(), files.end(), "mega.json"));

            // Test mega_core.json and remove it from the files vector
            ASSERT_TRUE(core.exists());
            ASSERT_TRUE(core.isFile());
            EXPECT_EQ(core.readFile(),
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
            files.erase(std::find(files.begin(), files.end(), "mega_core.json"));

            // Test upload.sh and remove it from the files vector
            ASSERT_TRUE(upload.exists());
            ASSERT_TRUE(upload.isFile());
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
            files.erase(std::find(files.begin(), files.end(), "upload.sh"));

            // Test serial.sh and remove it from the files vector
            ASSERT_TRUE(serial.exists());
            ASSERT_TRUE(serial.isFile());
            EXPECT_EQ(serial.readFile(),
                "#!/usr/bin/env bash\n"
                "\n"
                "picocom /dev/mega -b 115200 --echo\n"
            );
            files.erase(std::find(files.begin(), files.end(), "serial.sh"));

            // Test platformio.ini and remove it from the files vector
            ASSERT_TRUE(platformio.exists());
            ASSERT_TRUE(platformio.isFile());
            EXPECT_EQ(platformio.readFile(),
                "[platformio]\n"
                "lib_dir = /Robot/ArduinoLibraries\n"
                "env_default = mega\n"
                "\n"
                "[env:mega]\n"
                "platform = atmelavr\n"
                "framework = arduino\n"
                "board = mega\n"
                "upload_port = /dev/mega\n"
            );
            files.erase(std::find(files.begin(), files.end(), "platformio.ini"));

            // Make sure there aren't any extra files
            ASSERT_EQ(files.size(), 0);

            // Check the file permissions
            #ifndef _WIN32
            // TODO setGID special bit support
            // EXPECT_EQ(device_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                                        FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                                        FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            // EXPECT_EQ(source_folder.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                                        FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                                        FilePermissions::OtherRead | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(source.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(config.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(core.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                          FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                          FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            EXPECT_EQ(upload.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(serial.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                            FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                            FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);
            EXPECT_EQ(platformio.permissions(), FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                                FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                                FilePermissions::OtherRead);// | FilePermissions::OtherWrite);
            #endif

            // Cleanup
            fs::open("./test/mega").removeDirectoryRec();
        }
    }
}
