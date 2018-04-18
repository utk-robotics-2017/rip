#include "cmd_messenger/cmd_messenger.hpp"
#include "cmd_messenger/command.hpp"
#include "cmd_messenger/exceptions.hpp"

#include <memory>

#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

#include <fmt/format.h>

#include "mock_device.hpp"

#include <iostream>

using MockDevice = rip::cmdmessenger::mocks::MockDevice;

namespace rip
{
    namespace cmdmessenger
    {
        namespace tests
        {
            /**
             * To make debugging easier `byteStringToHexDebugString` converts a byte string to a human readable format.
             * If the byte is an ASCII character, it will be display as BYTE(CHAR). i.e. `0x41` would be displayed as `41(A)`.
             *
             * All of these unit tests use default values for the seperators, which are the following
             *   field_separator:   `2C(,)`
             *   command_separator: `3B(;)`
             *   escape_character:  `2F(/)`
             *
             * Ex 1:
             * A properly formed message always starts with the command id in ASCII and ends with the command separator.
             *   e.g. Sending a command with id 12 and no arguments would be sent as:
             *        `31(1) 32(2) 3B(;)`
             *
             * Ex 2:
             * A command message with arguments will have them inserted after the command id and before the command separator,
             *   separated by the field separator.
             *   e.g. Sending a command with id 7 and two character arguments ('a', 'd') would be sent as:
             *        `37(7) 2C(,) 61(a) 2C(,) 64(d) 3B(;)`
             *
             * Ex 3:
             * If an argument has more than one byte, it will be serialized with starting with the least significant byte.
             *   e.g. Sending a command with id 25 and one unsigned integer (0x1234) would be sent as:
             *        `32(2) 35(5) 2C(,) 34(4) 12 3B(;)`
             *
             * Ex 4:
             * If an argument byte is illegal (field separator `2C(,)`, command separator `3B(;)`, escape character `2F(/)`, or null character `00`),
             *   it will be escaped with the escape character `2F(/)`.
             *   e.g. Sending a command with id 42 and an unsigned long (0x2C3B2F00) would be sent as:
             *        `34(4) 32(2) 2C(,) 2F(/) 00 2F(/) 2F(/) 2F(/) 3B(;) 2F(/) 2C(,) 3B(;)`
             */

            TEST(CmdMessenger_docs, Ex1)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 12, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command));
                // 31(1) 32(2) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 3u);
                ASSERT_EQ(sent, "12;");
            }

            TEST(CmdMessenger_docs, Ex2)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::CharType, ArduinoCmdMessenger::CharType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 'a', 'd'));
                // 37(7) 2C(,) 61(a) 2C(,) 64(d) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u);
                ASSERT_EQ(sent, "7,a,d;");
            }

            TEST(CmdMessenger_docs, Ex3)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedIntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 25, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 0x1234));
                // 32(2) 35(5) 2C(,) 34(4) 12 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u);
                ASSERT_EQ(sent, fmt::format("25,{}{};", (char)0x34, (char)0x12));
            }

            TEST(CmdMessenger_docs, Ex4)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedLongType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 42, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 0x2C3B2F00));
                // 34(4) 32(2) 2C(,) 2F(/) 00 2F(/) 2F(/) 2F(/) 3B(;) 2F(/) 2C(,) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 12u);
                ASSERT_EQ(sent, fmt::format("42,/{}/{}/{}/{};", (char)0x00, (char)0x2F, (char)0x3B, (char)0x2C));
            }

            TEST(CmdMessenger_send, Empty)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command));
                // 37(7) 3B(;)
            }

            TEST(CmdMessenger_send, Int)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));
                // 37(7) 2C(,) 01 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<int16_t>(sent), 1);
            }

            TEST(CmdMessenger_send, UnsignedInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedIntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));
                // 37(7) 2C(,) 01 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 1u);
            }

            TEST(CmdMessenger_send, Long)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::LongType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));
                // 37(7) 2C(,) 01 2F(/) 00 2F(/) 00 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 10u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<int32_t>(sent), 1);
            }

            TEST(CmdMessenger_send, UnsignedLong)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedLongType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 123));
                // 37(7) 2C(,) 7B({) 2F(/) 00 2F(/) 00 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 10u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<uint32_t>(sent), 123u);
            }

            TEST(CmdMessenger_send, Float)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 2.0f));
                // 37(7) 2C(,) 2F(/) 00 2F(/) 00 2F(/) 00 40(@) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 10u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.0);
            }

            TEST(CmdMessenger_send, Double)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::DoubleType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 2.5));
                // 37(7) 2C(,) 2F(/) 00 2F(/) 00 20( ) 40(@) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 9u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.5);
            }

            TEST(CmdMessenger_send, Char)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::CharType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 'c'));
                // 37(7) 2C(,) 63(c) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 4u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<char>(sent), 'c');
            }

            TEST(CmdMessenger_send, String)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::StringType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, "Hello World"));
                // 37(7) 2C(,) 48(H) 65(e) 6C(l) 6C(l) 6F(o) 20( ) 57(W) 6F(o) 72(r) 6C(l) 64(d) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 14u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<std::string>(sent), "Hello World");
            }

            TEST(CmdMessenger_send, Boolean)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::BooleanType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, false));
                // 37(7) 2C(,) 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 5u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<char>(sent), false);
            }

            TEST(CmdMessenger_send, MultiInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 1, 2));
                // 37(7) 2C(,) 01 2F(/) 00 2C(,) 02 2F(/) 00 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 10u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<int16_t>(sent), 1);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<int16_t>(sent), 2);
            }

            TEST(CmdMessenger_send, MultiIntFloat)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                RIP_ASSERT_NO_THROW(cmd_messenger.send(device, command, 1, 2.0));
                // 37(7) 2C(,) 01 2F(/) 00 2C(,) 2F(/) 00 2F(/) 00 2F(/) 00 40(@) 3B(;)

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 14u);

                ASSERT_EQ(sent[0], '7');
                sent.erase(0, 2);
                ASSERT_EQ(device->fromBytes<int16_t>(sent), 1);
                sent.erase(0, 2);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.0);
            }

            TEST(CmdMessenger_send, FailedAcknowledgement)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();

                device->setAcknowledgeCorrectly(false);

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger; // Changed because errors now throw exception
                RIP_ASSERT_THROW(cmd_messenger.send(device, command, 1, 2.0), cmdmessenger::DeviceSentErrorResponse);
            }

            TEST(CmdMessenger_response, Int)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command, 1);
                // 37(7) 2C(,) 01 2F(/) 00 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{};", (char)0x10, (char)0x00));
                // 38(8) 2C(,) 10 2F(/) 00 3B(;)

                std::tuple<ArduinoCmdMessenger::IntegerType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::IntegerType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, UnsignedInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedIntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{};", (char)0x10, (char)0x00));
                // 38(8) 2C(,) 10 2F(/) 00 3B(;)

                std::tuple<ArduinoCmdMessenger::UnsignedIntegerType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::UnsignedIntegerType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, Long)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::LongType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{}/{}/{};", (char)0x10, (char)0x00, (char)0x00, (char)0x00));
                // 38(8) 2C(,) 10 2F(/) 00 2F(/) 00 2F(/) 00 3B(;)

                std::tuple<ArduinoCmdMessenger::LongType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::LongType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, UnsignedLong)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedLongType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{}/{}/{};", (char)0x10, (char)0x00, (char)0x00, (char)0x00));
                // 38(8) 2C(,) 10 2F(/) 00 2F(/) 00 2F(/) 00 3B(;)

                std::tuple<ArduinoCmdMessenger::UnsignedLongType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::UnsignedLongType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16u);
            }

            TEST(CmdMessenger_response, Float)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,/{}/{}{}{};", (char)0x00, (char)0x00, (char)0x20, (char)0x40));
                // 38(8) 2C(,) 2F(/) 00 2F(/) 00 20( ) 40(@) 3B(;)

                std::tuple<ArduinoCmdMessenger::FloatType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::FloatType>(command));
                ASSERT_FLOAT_EQ(std::get<0>(response_tuple), 2.5);
            }

            TEST(CmdMessenger_response, Double)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,/{}/{}{}{};", (char)0x00, (char)0x00, (char)0x20, (char)0x40));
                // 38(8) 2C(,) 2F(/) 00 2F(/) 00 20( ) 40(@) 3B(;)

                std::tuple<ArduinoCmdMessenger::DoubleType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::DoubleType>(command));
                ASSERT_FLOAT_EQ(std::get<0>(response_tuple), 2.5);
            }

            TEST(CmdMessenger_response, Char)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::CharType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse("8,c;");
                // 38(8) 2C(,) 63(c) 3B(;)

                std::tuple<ArduinoCmdMessenger::CharType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::CharType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 'c');
            }

            TEST(CmdMessenger_response, Boolean)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::BooleanType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{};", (char)0x01));
                // 38(8) 2C(,) 01 3B(;)

                std::tuple<ArduinoCmdMessenger::BooleanType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::BooleanType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), true);
            }

            TEST(CmdMessenger_response, String)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::StringType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse("8,Hello World;");
                // 38(8) 2C(,) 48(H) 65(e) 6C(l) 6C(l) 6F(o) 20( ) 57(W) 6F(o) 72(r) 6C(l) 64(d) 3B(;)

                std::tuple<ArduinoCmdMessenger::StringType> response_tuple;
                RIP_ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::StringType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), "Hello World");
            }

            TEST(CmdMessenger_response, IntInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{},{}/{};", (char)0x0C, (char)0x00, (char)0x0E, (char)0x00));
                // 38(8) 2C(,) 0C 2F(/) 00 2C(,) 0E 2F(/) 00 3B(;)

                std::tuple<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType> response_tuple;
                response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType>(command);
                ASSERT_EQ(std::get<0>(response_tuple), 12);
                ASSERT_EQ(std::get<1>(response_tuple), 14);
            }

            TEST(CmdMessenger_response, IntFloat)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);
                // 37(7) 3B(;)

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                device->setResponse(fmt::format("8,{}/{},{}{}{}{};", (char)0x0C, (char)0x00, (char)0x66, (char)0x66, (char)0x66, (char)0x40));
                // 38(8) 2C(,) 0C 2F(/) 00 2C(,) 66(f) 66(f) 66(f) 40(@) 3B(;)

                std::tuple<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType> response_tuple;
                response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>(command);
                ASSERT_EQ(std::get<0>(response_tuple), 12);
                ASSERT_FLOAT_EQ(std::get<1>(response_tuple), 3.6);
            }
        }
    }
}
