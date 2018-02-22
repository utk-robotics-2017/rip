#include "cmd_messenger/cmd_messenger.hpp"
#include "cmd_messenger/command.hpp"
#include "cmd_messenger/exceptions.hpp"

#include <memory>

#include <gtest/gtest.h>

#include "mock_device.hpp"

using MockDevice = rip::cmdmessenger::mocks::MockDevice;

namespace rip
{
    namespace cmdmessenger
    {
        namespace tests
        {
            TEST(CmdMessenger_send, Empty)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command));
            }

            TEST(CmdMessenger_send, Int)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u); // 1 2 byte int, 1 2 byte int and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<int16_t>(sent), 1);
            }

            TEST(CmdMessenger_send, UnsignedInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedIntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 6u); // 1 2 byte int, 1 2 byte int and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 1u);
            }

            TEST(CmdMessenger_send, Long)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::LongType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 1));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 8u); // 1 2 byte int, 1 4 byte string and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<int32_t>(sent), 1);
            }

            TEST(CmdMessenger_send, UnsignedLong)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedLongType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 123));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 8u); // 1 2 byte int, 1 4 byte int and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<uint32_t>(sent), 123u);
            }

            TEST(CmdMessenger_send, Float)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 2.0f));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 8u); // 1 2 byte int, 1 4 byte float and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.0);
            }

            TEST(CmdMessenger_send, Double)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::DoubleType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 2.5));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 8u); // 1 2 byte int, 1 4 byte float and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.5);
            }

            TEST(CmdMessenger_send, Char)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::CharType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 'c'));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 5u); // 1 2 byte int, 1 1 byte char and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<char>(sent), 'c');
            }

            TEST(CmdMessenger_send, String)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::StringType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, "Hello World"));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 15u); // 1 2 byte int, 1 12 byte string and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<std::string>(sent), "Hello World");
            }

            TEST(CmdMessenger_send, Boolean)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::BooleanType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, false));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 5u); // 1 2 byte int, 1 1 byte char and 2 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<char>(sent), false);
            }

            TEST(CmdMessenger_send, MultiInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 1, 2));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 9u); // 3 2 byte ints and 3 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 1u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 2u);
            }

            TEST(CmdMessenger_send, MultiIntFloat)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_NO_THROW(cmd_messenger.send(device, command, 1, 2.0));

                std::string sent = device->getLastSent();
                ASSERT_EQ(sent.size(), 11u); //2 2 byte ints, 1 4 byte float and 3 1 byte chars

                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 7u);
                sent.erase(0, 1);
                ASSERT_EQ(device->fromBytes<uint16_t>(sent), 1u);
                sent.erase(0, 1);
                ASSERT_FLOAT_EQ(device->fromBytes<float>(sent), 2.0);
            }

            TEST(CmdMessenger_send, FailedAcknowledgement)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();

                std::string argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();

                device->setAcknowledgeCorrectly(false);

                std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

                ArduinoCmdMessenger cmd_messenger;
                ASSERT_THROW(cmd_messenger.send(device, command, 1, 2.0), cmdmessenger::IncorrectAcknowledgementCommand);
            }

            TEST(CmdMessenger_response, Int)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.
                send(device, send_command, 1);


                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(16) + ';';
                device->
                setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::IntegerType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::IntegerType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, UnsignedInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);


                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedIntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<uint16_t>(16) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::UnsignedIntegerType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::UnsignedIntegerType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, Long)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);


                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::LongType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int32_t>(16) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::LongType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::LongType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16);
            }

            TEST(CmdMessenger_response, UnsignedLong)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);


                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::UnsignedLongType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<uint32_t>(16) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::UnsignedLongType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::UnsignedLongType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 16u);
            }

            TEST(CmdMessenger_response, Float)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<float>(2.5) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::FloatType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::FloatType>(command));
                ASSERT_FLOAT_EQ(std::get<0>(response_tuple), 2.5);
            }

            TEST(CmdMessenger_response, Double)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<float>(2.5) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::DoubleType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::DoubleType>(command));
                ASSERT_FLOAT_EQ(std::get<0>(response_tuple), 2.5);
            }

            TEST(CmdMessenger_response, Char)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::CharType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<char>('c') + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::CharType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::CharType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), 'c');
            }

            TEST(CmdMessenger_response, Boolean)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::BooleanType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<bool>(true) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::BooleanType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::BooleanType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), true);
            }

            TEST(CmdMessenger_response, String)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;

                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::StringType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + "Hello World" + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::StringType> response_tuple;
                ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::StringType>(command));
                ASSERT_EQ(std::get<0>(response_tuple), "Hello World");
            }

            TEST(CmdMessenger_response, IntInt)
            {
                std::shared_ptr<MockDevice> device = std::make_shared<MockDevice>();
                ArduinoCmdMessenger cmd_messenger;


                std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
                std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
                cmd_messenger.send(device, send_command);


                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::IntegerType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(12) + ',' + device->toBytes<int16_t>(14) + ';';
                device->setResponse(set_response);

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

                std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>();
                std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

                std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(12) + ',' + device->toBytes<float>(3.6) + ';';
                device->setResponse(set_response);

                std::tuple<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType> response_tuple;
                response_tuple = cmd_messenger.receive<ArduinoCmdMessenger::IntegerType, ArduinoCmdMessenger::FloatType>(command);
                ASSERT_EQ(std::get<0>(response_tuple), 12);
                ASSERT_FLOAT_EQ(std::get<1>(response_tuple), 3.6);
            }
        }
    }
}
