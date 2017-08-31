#include <cmd_messenger.hpp>
#include <command.hpp>
#include <device.hpp>
#include <exceptions.hpp>

#include <memory>

#include <gtest/gtest.h>

#include "mock_device.hpp"

using Device = cmdmessengermocks::MockDevice;
using Command = cmdmessenger::Command;
using ArduinoCmdMessenger = cmdmessenger::ArduinoCmdMessenger;

namespace cmdmessengertest
{
    TEST(CmdMessenger, EmptySend)
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger.send(device, command));
    }

    TEST(CmdMessenger, IntSend)
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<int>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger.send(device, command,1));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent.size(),6); // 1 2 byte int, 1 2 byte int and 2 1 byte chars

        ASSERT_EQ(device->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<int16_t>(sent),
                1);
    }

    TEST(CmdMessenger, UnsignedIntSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<unsigned int>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    1));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                6); // 1 2 byte int, 1 2 byte int and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                1);
    }

    TEST(CmdMessenger, LongSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<long>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    1));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                8); // 1 2 byte int, 1 4 byte string and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<int32_t>(sent),
                1);
    }

    TEST(CmdMessenger, UnsignedLongSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<unsigned long>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    123));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                8); // 1 2 byte int, 1 4 byte int and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<uint32_t>(sent),
                123);
    }

    TEST(CmdMessenger, FloatSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<float>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    2.0f));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                8); // 1 2 byte int, 1 4 byte float and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_FLOAT_EQ(device
                ->
                fromBytes<float>(sent),
                2.0);
    }

    TEST(CmdMessenger, DoubleSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<double>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    2.5));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                8); // 1 2 byte int, 1 4 byte float and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_FLOAT_EQ(device
                ->
                fromBytes<float>(sent),
                2.5);
    }

    TEST(CmdMessenger, CharSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<char>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    'c'));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                5); // 1 2 byte int, 1 1 byte char and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<char>(sent),
                'c');
    }

    TEST(CmdMessenger, StringSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<std::string>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    "Hello World"));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                15); // 1 2 byte int, 1 12 byte string and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<std::string>(sent),
                "Hello World");
    }

    TEST(CmdMessenger, BooleanSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<bool>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    false));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                5); // 1 2 byte int, 1 1 byte char and 2 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<char>(sent),
                false);
    }

    TEST(CmdMessenger, MultiIntSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<int, int>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    1, 2));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                9); // 3 2 byte ints and 3 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                1);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                2);
    }

    TEST(CmdMessenger, MultiIntFloatSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<int, float>();

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_NO_THROW(cmd_messenger
                .
                send(device, command,
                    1, 2.0));

        std::string sent = device->getLastSent();
        ASSERT_EQ(sent
                .

                size(),

                11); //2 2 byte ints, 1 4 byte float and 3 1 byte chars

        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                7);
        sent.erase(0,1);
        ASSERT_EQ(device
                ->
                fromBytes<uint16_t>(sent),
                1);
        sent.erase(0,1);
        ASSERT_FLOAT_EQ(device
                ->
                fromBytes<float>(sent),
                2.0);
    }

    TEST(CmdMesenger, FailedAcknowledgementSend
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();

        std::string argument_types = ArduinoCmdMessenger::makeArgumentString<int, float>();

        device->setAcknowledgeCorrectly(false);

        std::shared_ptr<Command> command = std::make_shared<Command>("kCommand", 7, argument_types);

        ArduinoCmdMessenger cmd_messenger;
        ASSERT_THROW(cmd_messenger
                .
                send(device, command,
                    1, 2.0), cmdmessenger::IncorrectAcknowledgementCommand);
    }

    TEST(CmdMessenger, IntResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;

        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString<int>();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command,
                    1);


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<int>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(16) + ';';
        device->
            setResponse(set_response);

        std::tuple<int> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<int>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                16);
    }

    TEST(CmdMessenger, UnsignedIntResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<unsigned int>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<uint16_t>(16) + ';';
        device->
            setResponse(set_response);

        std::tuple<unsigned int> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<unsigned int>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                16);
    }

    TEST(CmdMessenger, LongResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<long>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<int32_t>(16) + ';';
        device->
            setResponse(set_response);

        std::tuple<long> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<long>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                16);
    }

    TEST(CmdMessenger, UnsignedLongResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<unsigned long>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<uint32_t>(16) + ';';
        device->
            setResponse(set_response);

        std::tuple<unsigned long> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<unsigned long>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                16);
    }

    TEST(CmdMessenger, FloatResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<float>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<float>(2.5) + ';';
        device->
            setResponse(set_response);

        std::tuple<float> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<float>(command)
                );
        ASSERT_FLOAT_EQ(std::get<0>(response_tuple),
                2.5);
    }

    TEST(CmdMessenger, DoubleResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<float>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<float>(2.5) + ';';
        device->
            setResponse(set_response);

        std::tuple<double> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<double>(command)
                );
        ASSERT_FLOAT_EQ(std::get<0>(response_tuple),
                2.5);
    }

    TEST(CmdMessenger, CharResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<char>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<char>('c') + ';';
        device->
            setResponse(set_response);

        std::tuple<char> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<char>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                'c');
    }

    TEST(CmdMessenger, BooleanResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<bool>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + device->toBytes<bool>(true) + ';';
        device->
            setResponse(set_response);

        std::tuple<bool> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<bool>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                true);
    }

    TEST(CmdMessenger, StringResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<std::string>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response = device->toBytes<uint16_t>(8) + ',' + "Hello World" + ';';
        device->
            setResponse(set_response);

        std::tuple<std::string> response_tuple;
        ASSERT_NO_THROW(response_tuple = cmd_messenger.receive<std::string>(command)
                );
        ASSERT_EQ(std::get<0>(response_tuple),
                "Hello World");
    }

    TEST(CmdMessenger, IntIntResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<int, int>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response =
            device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(12) + ',' + device->toBytes<int16_t>(14) + ';';
        device->
            setResponse(set_response);

        std::tuple<int, int> response_tuple;
        response_tuple = cmd_messenger.receive<int, int>(command);
        ASSERT_EQ(std::get<0>(response_tuple),
                12);
        ASSERT_EQ(std::get<1>(response_tuple),
                14);
    }

    TEST(CmdMessenger, IntFloatResponse
        )
    {
        std::shared_ptr<Device> device = std::make_shared<Device>();
        ArduinoCmdMessenger cmd_messenger;


        std::string send_argument_types = ArduinoCmdMessenger::makeArgumentString();
        std::shared_ptr<Command> send_command = std::make_shared<Command>("kCommand", 7, send_argument_types);
        cmd_messenger.
            send(device, send_command
                );


        std::string arguments_types = ArduinoCmdMessenger::makeArgumentString<int, float>();
        std::shared_ptr<Command> command = std::make_shared<Command>("kCommandResult", 8, arguments_types);

        std::string set_response =
            device->toBytes<uint16_t>(8) + ',' + device->toBytes<int16_t>(12) + ',' + device->toBytes<float>(3.6) + ';';
        device->
            setResponse(set_response);

        std::tuple<int, float> response_tuple;
        response_tuple = cmd_messenger.receive<int, float>(command);
        ASSERT_EQ(std::get<0>(response_tuple),
                12);
        ASSERT_FLOAT_EQ(std::get<1>(response_tuple),
                3.6);
    }}
