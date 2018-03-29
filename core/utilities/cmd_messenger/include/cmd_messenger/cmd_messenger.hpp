#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */

#ifndef CMD_MESSENGER_HPP
#define CMD_MESSENGER_HPP

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <tuple>
#include <iostream>

#include <fmt/format.h>

#include <cmd_messenger/exceptions.hpp>
#include "command.hpp"
#include "device.hpp"

namespace rip
{
    namespace cmdmessenger
    {
        /**
         * @class CmdMessenger
         *
         *  @brief Packs a series of arguments in to a command message that can be sent
         *
         * @tparam T_StringType The type for representing a string (may not work with any other than the default type)
         * @tparam T_IntegerType The type for representing a cross-platform integer
         * @tparam T_UnsignedIntegerType
         * @tparam T_LongType
         * @tparam T_UnsignedLongType
         * @tparam T_FloatType
         * @tparam T_DoubleType
         * @tparam T_BooleanType
         * @tparam T_CharType
         */
        template <
            typename T_StringType = std::string,
            typename T_IntegerType = int,
            typename T_UnsignedIntegerType = unsigned int,
            typename T_LongType = long,
            typename T_UnsignedLongType = unsigned long,
            typename T_FloatType = float,
            typename T_DoubleType = double,
            typename T_BooleanType = bool,
            typename T_CharType = char
            >
        class CmdMessenger
        {
        public:

            typedef T_StringType          StringType         ;
            typedef T_IntegerType         IntegerType        ;
            typedef T_UnsignedIntegerType UnsignedIntegerType;
            typedef T_LongType            LongType           ;
            typedef T_UnsignedLongType    UnsignedLongType   ;
            typedef T_FloatType           FloatType          ;
            typedef T_DoubleType          DoubleType         ;
            typedef T_BooleanType         BooleanType        ;
            typedef T_CharType            CharType           ;


            /**
             * @brief Creates an empty string for when makeArgumentsString is called with no template arguments
             *
             * @note can be called as makeArgumentString() or makeArgumentString<>()
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             */
            template<typename... Args>
            static typename std::enable_if<sizeof...(Args) == 0, const std::string>::type makeArgumentString()
            {
                return "";
            }

            /**
             * @brief Creates a string with each character representing an argument type using the template arguments
             *makeArgumentString
             * @note Does this recursively
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             */
            template<typename T, typename... Args>
            static std::string makeArgumentString()
            {
                return makeArgumentChar<T>() + makeArgumentString<Args...>();
            }

            /**
             * @brief Constructor
             *
             * @param field_separator The character used to separate fields in a single command
             * @param command_separator The character used to separate commands
             * @param escape_character The character used to escape the separators
             */
            CmdMessenger(char field_separator = ',', char command_separator = ';', char escape_character = '/')
                : m_max_response_length(1023)
                , m_field_separator(field_separator)
                , m_command_separator(command_separator)
                , m_escape_character(escape_character)
                , m_last_device(nullptr)
            {
            }

            /**
             * @brief Send the command to a device with the specified arguments
             *
             * @param device The device to send the command to
             * @param command The command to send
             * @param args The list of arguments to send
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             */
            template <typename... Args>
            void send(std::shared_ptr<Device> device, std::shared_ptr<Command> command, Args... args)
            {
                std::string debugString; // HACK
                if (device == nullptr)
                {
                    throw EmptyDevice();
                }

                if (command->getId() == "")
                {
                    throw EmptyCommand();
                }

                std::cout << fmt::format("Command->getEnum(): {} (aka {})", command->getEnum(), command->getId()) << std::endl;

                // Get the argument types
                std::string argument_types = command->getArgumentTypes();
                const std::size_t value = sizeof...(Args);
                if (value != argument_types.size())
                {
                    throw IncorrectArgumentListSize();
                }

                debugString = byteStringToHexDebugString(std::to_string(command->getEnum()));
                std::cout << fmt::format("to_str(command->getEnum()) bytes: {}", debugString) << std::endl;
                // std::cout << toBytes<int, T_IntegerType>(command->getEnum()) << std::endl;

                // Pack the command to send
                std::string message = std::to_string(command->getEnum()) + static_cast<T_CharType>(m_field_separator);

                std::tuple<Args...> args_tuple(args...);

                if (sizeof...(args) > 0)
                {
                    message += tupleToBytes<0, Args...>(argument_types, args_tuple);
                    message.back() = static_cast<T_CharType>(m_command_separator);
                }
                else
                {
                    message += static_cast<T_CharType>(m_command_separator);
                }

                // Send the message
                debugString = byteStringToHexDebugString(message);
                std::cout << fmt::format("Device->write bytes: {}", debugString) << std::endl;
                device->write(message);

                // HACK
                device->setTimeout(units::s * 3);

                // Check Acknowledgement
                std::string acknowledgement = device->readline(m_max_response_length, std::string(1, m_command_separator));

                if (acknowledgement.length() == 0)
                {
                  throw EmptyDeviceResponse(fmt::format("did not receive any bytes from the device, timeout or crash?"));
                }

                debugString = byteStringToHexDebugString(acknowledgement);
                std::cout << fmt::format("Device->readline bytes: {}", debugString) << std::endl;

                handleAck(acknowledgement, command);

                m_last_device = device;
            }

            /**
             * @brief Checks if the sent command was properly acknowledged
             *
             * @param acknowledgment The string returned by the device after sending a command
             * @param command The command metadata for the command sent
             *
             * @exception IncorrectFieldSeparator Thrown if the field separator in between the acknowledge command and the sent command is incorrect
             * @exception IncorrectCommandSeparator Thrown if the acknowledgment does not end with the correct command separator
             * @exception IncorrectAcknowledgementCommand Thrown if the command acknowledged is not the one previously sent
             */
            void handleAck(std::string& acknowledgement, std::shared_ptr<Command> command)
            {
                std::string debugString = byteStringToHexDebugString(acknowledgement);
                std::cout << fmt::format("Ack Str: {}", debugString) << std::endl;

                // First part should be the acknowledgment id
                // (a two-byte integer by default)
                // note that fromBytes consumes and modifies the target
                // T_CharType acknowledgement_id = fromBytes<T_CharType>(acknowledgement);
                T_IntegerType acknowledgement_id = std::stoi(acknowledgement.substr(0, acknowledgement.find(m_field_separator)), nullptr);
                acknowledgement.erase(0, acknowledgement.find(m_field_separator));

                // kAcknowledge will always be zero
                if (acknowledgement_id == 1)
                {
                  debugString = byteStringToHexDebugString(acknowledgement);
                  throw DeviceSentErrorResponse(fmt::format("handleAck: device returned 1:kError: {}", debugString));
                }
                else if (acknowledgement_id != 0)
                {
                  throw IncorrectAcknowledgementCommand(fmt::format("handleAck: Acknowledge command incorrect, got {} instead of zero.", acknowledgement_id));
                }

                // XXX
                debugString = byteStringToHexDebugString(acknowledgement);
                std::cout << fmt::format("After checking ack_id: {}", debugString) << std::endl;
                // Then the field separator
                if (acknowledgement[0] != m_field_separator)
                {
                   std::cout << fmt::format("Bad field_separator: Wanted '{}' but got '{}' !", m_field_separator, acknowledgement[0]) << std::endl;
                    throw IncorrectFieldSeparator(fmt::format("Wanted '{}' but got '{}' !", m_field_separator, acknowledgement[0]));
                }
                acknowledgement.erase(0, 1);

                debugString = byteStringToHexDebugString(acknowledgement);
                std::cout << fmt::format("After checking field_separator: {}", debugString) << std::endl;

                // Then the command sent
                T_IntegerType acknowledge_command = fromBytes<T_IntegerType>(acknowledgement);
                // NOTE fromBytes consumes the data from the string!
                if ( acknowledge_command != static_cast<T_IntegerType>(command->getEnum()) )
                {
                  std::cout << fmt::format("Acknowledgement command {:d} is not the same as the current command {:d}", acknowledge_command, command->getEnum()) << std::endl;
                    throw IncorrectAcknowledgementCommand(fmt::format("Acknowledgement command {} is not the same as the current command {}", acknowledge_command, command->getEnum()));
                }
                if (acknowledgement[0] != m_command_separator)
                {
                  std::cout << fmt::format("Wanted '{}' but got '{}' !", m_command_separator, acknowledgement[0]) << std::endl;
                    throw IncorrectCommandSeparator(fmt::format("Wanted '{}' but got '{}' !", m_command_separator, acknowledgement[0]));
                }

                std::cout << fmt::format("Successfully acknowledged command {}.", command->getId()) << std::endl;
            }

            /**
             * @brief Receive a response from the device that a command was last sent to
             *
             * @tparam Args The parameter pack of types to receive
             *
             * @param command The command to receive
             *
             * @returns A tuple that contains
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             *
             * @todo(Andrew): add exceptions
             */
            template<typename... Args>
            std::tuple<Args...> receive(std::shared_ptr<Command> command)
            {
                // \todo(Andrew): add comment to this function
                if (!m_last_device)
                {
                    throw NoLastDeviceException();
                }


                const std::size_t num_arguments = sizeof...(Args);

                std::string argument_types = command->getArgumentTypes();

                if (num_arguments != argument_types.size())
                {
                    throw IncorrectArgumentListSize();
                }

                // Unpack the message
                std::string response = m_last_device->readline(m_max_response_length, std::string(1, m_command_separator));

                if (response.size() == 0)
                {
                   throw EmptyDeviceResponse("In CmdMessenger->receive(), device->receive() did not return anything. Timeout?");
                }

                // Check that response command is correct
                T_IntegerType response_command_enum = std::stoi(response.substr(0, response.find(m_field_separator)), nullptr);
                response.erase(0, response.find(m_field_separator));

                if (response_command_enum != command->getEnum())
                {
                    throw IncorrectResponseCommand();
                }

                if (response[0] != m_field_separator)
                {
                    throw IncorrectFieldSeparator();
                }
                response.erase(0, 1);

                std::tuple<Args...> return_values;
                bytesToTuple<0, Args...>(argument_types, response, return_values);

                m_last_device.reset();

                return return_values;
            }

        private:

            /**
             *  @brief Any type that cannot be implicitly converted to string will use this function and throw an exception
             *
             *  @param str A value that cannot be converted to a string
             *
             *  @exception Unconvertible Argument
             */
            template<typename T>
            std::string toBytesString(const T& str)
            {
                throw UnconvertibleArgument();
                return "";
            }

            /**
             * @brief Effectively filters what can be implicitly converted to string
             *
             * @param str The converted string
             *
             * @return A string
             */
            std::string toBytesString(const std::string& str)
            {
                return str;
            }

            // HACK
            using byte = unsigned char ;

            template< typename T > std::array< byte, sizeof(T) >  to_bytes( const T& object )
            {
                std::array< byte, sizeof(T) > bytes;

                const byte* begin = reinterpret_cast< const byte* >( std::addressof(object) ) ;
                const byte* end = begin + sizeof(T) ;
                std::copy( begin, end, std::begin(bytes) ) ;

                return bytes;
            }

            /**
             * @brief Convert a type into bytes
             *
             * @tparam From The current value of the type to convert to bytes
             * @tparam To The type that is specified for the device
             *
             * @param f The value to convert into bytes
             *
             * @returns A string (as a list of bytes)
             *
             * @note This function will only compile if it is possible to convert from type From to type To
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             * @ref https://en.cppreference.com/w/cpp/types/is_convertible
             * @ref https://en.cppreference.com/w/cpp/types/is_same
             */
            template<typename From, typename To>
            typename std::enable_if<std::is_convertible<From, To>::value, std::string>::type toBytes(const From& f)
            {
                /*
                To t = static_cast<To>(f);

                if (std::is_same<To, T_StringType>::value)
                {
                    return toBytesString(t);
                }

                char* byte_pointer = reinterpret_cast<char*>(&t);
                std::string rv;
                for (size_t i = 0; i < sizeof(t); i++)
                {
                    // Add the escape character
                    if (*byte_pointer == m_field_separator ||
                            *byte_pointer == m_command_separator ||
                            *byte_pointer == m_escape_character)
                    {
                        rv.push_back(static_cast<T_CharType>(m_escape_character));
                    }
                    rv.push_back(*byte_pointer);
                    byte_pointer ++;
                }
                return rv;
                //*/
                To t = static_cast<To>(f);

                if (std::is_same<To, T_StringType>::value)
                {
                    return toBytesString(t);
                }

                std::string rv;
                for (auto a_chr : to_bytes(t))
                {
                    char n_chr = static_cast<char>(a_chr);
                    rv += n_chr;
                }
                std::reverse(rv.begin(), rv.end());
                return rv;
            }

            /**
             * @brief Filters out the types that cannot be converted and throws an exception
             *
             * @param t The value to convert into bytes
             *
             * @returns A string (as a list of chars)
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             * @ref https://en.cppreference.com/w/cpp/types/is_convertible
             */
            template<typename From, typename To>
            typename std::enable_if<not std::is_convertible<From, To>::value, std::string>::type toBytes(const From& f)
            {
                throw UnconvertibleArgument();
                return "";
            }

            /**
             * @brief Returns an empty string as the last part of the recursion or if tupleToBytes is called with no arguments
             *
             * @tparam I Index of the element of the parameter pack (In this case is equal to the size of the parameter pack)
             * @tparam Args The parameter pack types
             *
             * @param args_tuple A tuple of arguments
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             */
            template<std::size_t I = 0, typename... Args>
            typename std::enable_if<I == sizeof...(Args), std::string>::type tupleToBytes(const std::string& argument_types, const std::tuple<Args...>& args_tuple)
            {
                return "";
            }

            /**
             * @brief Converts a tuple of arguments to a string
             *
             * @tparam I Index of the element of the parameter pack (In this case is equal to the size of the parameter pack)
             * @tparam Args The parameter pack types
             *
             * @param args_tuple A tuple of arguments
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             *
             * @note This is done recursively
             */
            template<std::size_t I = 0, typename... Args>
            typename std::enable_if < I < sizeof...(Args), std::string>::type tupleToBytes(const std::string& argument_types, const std::tuple<Args...>& args_tuple)
            {
                std::string message;

                // Get the argument at the index I
                auto arg = std::get<I>(args_tuple);

                // Add the bytes for the current argument based on the argument type specified
                switch (argument_types[I])
                {
                    case m_integer_key:
                        message = toBytes<decltype(arg), T_IntegerType>(arg);
                        break;
                    case m_unsigned_integer_key:
                        message = toBytes<decltype(arg), T_UnsignedIntegerType>(arg);
                        break;
                    case m_long_key:
                        message = toBytes<decltype(arg), T_LongType>(arg);
                        break;
                    case m_unsigned_long_key:
                        message = toBytes<decltype(arg), T_UnsignedLongType>(arg);
                        break;
                    case m_float_key:
                        message = toBytes<decltype(arg), T_FloatType>(arg);
                        break;
                    case m_double_key:
                        message = toBytes<decltype(arg), T_DoubleType>(arg);
                        break;
                    case m_char_key:
                        message = toBytes<decltype(arg), T_CharType>(arg);
                        break;
                    case m_bool_key:
                        message = toBytes<decltype(arg), T_BooleanType>(arg);
                        break;
                    case m_string_key:
                        message = toBytes<decltype(arg), T_StringType>(arg);
                        break;
                    default:
                        throw UnknownArgument();
                }

                // Add the byte for field separator
                message += static_cast<T_CharType>(m_field_separator);

                // Recurse through future elements
                message += tupleToBytes < I + 1, Args... > (argument_types, args_tuple);

                // Remove comma and add semi-colon

                return message;
            }

            /**
             * @brief Pull a string out of the command
             *
             * @param message A string as a representation of bytes
             *
             * @note This version is used when the string can convert directly to the type
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             * @ref https://en.cppreference.com/w/cpp/types/is_convertible
             */
            template<typename Desired>
            typename std::enable_if<std::is_convertible<std::string, Desired>::value, Desired>::type fromBytesString(std::string& message)
            {
                std::string rv;
                size_t comma_position = message.find(m_field_separator);
                if (comma_position != std::string::npos)
                {
                    rv = message.substr(0, comma_position);
                    message.erase(0, comma_position);
                    return rv;
                }
                rv = message.substr(0, message.size() - 1); // remove command separator
                message.erase(0, message.size() - 1);
                return rv;
            }

            /**
             * @brief The version of the fromBytesString
             *
             * @param message A string as a representation of bytes
             *
             * @note This version is used when the string cannot convert directly to the type
             *
             * @exception UnconvertibleArgument Thrown because conversion is not possible
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             * @ref https://en.cppreference.com/w/cpp/types/is_convertible
             */
            template<typename Desired>
            typename std::enable_if<not std::is_convertible<std::string, Desired>::value, Desired>::type fromBytesString(std::string& message)
            {
                throw UnconvertibleArgument();
                return Desired();
            }

            /**
             * @brief Converts part of a string (as a list of bytes) into an object of type T
             *
             * @tparam T The type that should be pulled from the list of bytes
             *
             * @param the string from which the value is pulled
             *
             * @returns The value of T type
             */
            template<typename T>
            T fromBytes(std::string& message)
            {
                T rv;
                char* byte_pointer = reinterpret_cast<char*>(&rv);
                uint16_t escape_chars_skipped = 0;
                for (size_t i = 0; i < (sizeof(rv) + escape_chars_skipped); i++)
                {
                    // Skip the escape character
                    if (message[i] == m_escape_character)
                    {
                        i++;
                        escape_chars_skipped++;
                    }
                    *byte_pointer = message[i];
                    byte_pointer ++;
                }
                message.erase(0, sizeof(rv)+escape_chars_skipped);
                // reverse the return val
                // char *Tstart = reinterpret_cast<char*>(&rv), *Tend = Tstart + sizeof(rv);
                // std::reverse(Tstart, Tend);
                return rv;
            }

            /**
             * @brief Last step of the recursion (shouldn't actually be call)
             *
             *
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             */
            template<size_t I = 0, typename... Args>
            typename std::enable_if< I == sizeof...(Args), void>::type bytesToTuple(const std::string& argument_types, std::string& str, std::tuple<Args...>& args)
            {
                return;
            }

            /**
             * @brief
             *
             * @ref http://en.cppreference.com/w/cpp/language/parameter_pack
             * @ref http://en.cppreference.com/w/cpp/utility/tuple
             * @ref http://en.cppreference.com/w/cpp/types/enable_if
             */
            template<size_t I = 0, typename... Args>
            typename std::enable_if < I < sizeof...(Args), void>::type bytesToTuple(const std::string& argument_types, std::string& str, std::tuple<Args...>& args)
            {
                auto& arg = std::get<I>(args);

                switch (argument_types[I])
                {
                    case m_integer_key:
                        arg = fromBytes<T_IntegerType>(str);
                        break;
                    case m_unsigned_integer_key:
                        arg = fromBytes<T_UnsignedIntegerType>(str);
                        break;
                    case m_long_key:
                        arg = fromBytes<T_LongType>(str);
                        break;
                    case m_unsigned_long_key:
                        arg = fromBytes<T_UnsignedLongType>(str);
                        break;
                    case m_float_key:
                        arg = fromBytes<T_FloatType>(str);
                        break;
                    case m_double_key:
                        arg = fromBytes<T_DoubleType>(str);
                        break;
                    case m_char_key:
                        arg = fromBytes<T_CharType>(str);
                        break;
                    case m_bool_key:
                        arg = fromBytes<T_BooleanType>(str);
                        break;
                    case m_string_key:
                        arg = fromBytesString<typename std::remove_reference<decltype(arg)>::type>(str);
                        break;
                    default:
                        throw UnknownArgument();
                }


                if (I < sizeof...(Args) - 1)
                {
                    if (str[0] != m_field_separator)
                    {
                        throw IncorrectFieldSeparator();
                    }
                    str.erase(0, 1);
                    bytesToTuple < I + 1, Args... > (argument_types, str, args);
                }
                else
                {
                    if (str[0] != m_command_separator)
                    {
                        throw IncorrectCommandSeparator();
                    }
                }
            }


            /**
             *  @brief Returns the character for integers
             *
             *  @note This function is only called if the template parameter is an `int`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_IntegerType>::value, char>::type makeArgumentChar()
            {
                return m_integer_key;
            }

            /**
             *  @brief Returns the character for unsigned integers
             *
             *  @note This function is only called if the template parameter is an `unsigned int`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_UnsignedIntegerType>::value, char>::type makeArgumentChar()
            {
                return m_unsigned_integer_key;
            }

            /**
             *  @brief Returns the character for longs
             *
             *  @note This function is only called if the template parameter is a `long`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_LongType>::value, char>::type makeArgumentChar()
            {
                return m_long_key;
            }

            /**
             *  @brief Returns the character for unsigned longs
             *
             *  @note This function is only called if the template parameter is an `unsigned long`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_UnsignedLongType>::value, char>::type makeArgumentChar()
            {
                return m_unsigned_long_key;
            }

            /**
             *  @brief Returns the character for floats
             *
             *  @note This function is only called if the template parameter is a `float`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_FloatType>::value, char>::type makeArgumentChar()
            {
                return m_float_key;
            }

            /**
             *  @brief Returns the character for doubles
             *
             *  @note This function is only called if the template parameter is a `double`
             */
            template<typename T>
            static typename std::enable_if < std::is_same<T, T_DoubleType>::value&& !std::is_same<T_FloatType, T_DoubleType>::value, char >::type makeArgumentChar()
            {
                return m_double_key;
            }

            /**
             *  @brief Returns the character for chars
             *
             *  @note This function is only called if the template parameter is a `char`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_CharType>::value, char>::type makeArgumentChar()
            {
                return m_char_key;
            }

            /**
             *  @brief Returns the character for strings
             *
             *  @note This function is only called if the template parameter is a `string`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_StringType>::value, char>::type makeArgumentChar()
            {
                return m_string_key;
            }

            /**
             *  @brief Returns the character for bools
             *
             *  @note This function is only called if the template parameter is a `bool`
             */
            template<typename T>
            static typename std::enable_if< std::is_same<T, T_BooleanType>::value, char>::type makeArgumentChar()
            {
                return m_bool_key;
            }

            /*
             * If the makeArgumentChar function is called with a template argument that is not one of the above then it will not compile
             */

            std::string byteStringToHexDebugString(std::string input)
            {
                std::string rv = "";

                for (char c : input)
                {
                    rv += fmt::format("{:02X}", c);

                    if (c >= 32 && c <= 126)
                    {
                        rv += fmt::format("({})", c);
                    }

                    rv += " ";
                }

                if (rv.size() > 0)
                {
                    rv.pop_back();
                }

                return rv;
            }

            static constexpr char m_integer_key = 'i';
            static constexpr char m_unsigned_integer_key = 'u';
            static constexpr char m_long_key = 'l';
            static constexpr char m_unsigned_long_key = 'U';
            static constexpr char m_float_key = 'f';
            static constexpr char m_double_key = 'd';
            static constexpr char m_char_key = 'c';
            static constexpr char m_string_key = 's';
            static constexpr char m_bool_key = 'b';

            std::size_t m_max_response_length;

            char m_field_separator;
            char m_command_separator;
            char m_escape_character;

            std::shared_ptr<Device> m_last_device;
        };

        /**
         * @typedef ArduinoCmdMessenger CmdMessenger with all the types that an Arduino uses
         */
        using ArduinoCmdMessenger = CmdMessenger <
                                    /* T_StringType                = */ std::string,
                                    /* class T_IntegerType         = */ int16_t,
                                    /* class T_UnsignedIntegerType = */ uint16_t,
                                    /* class T_LongType            = */ int32_t,
                                    /* class T_UnsignedLongType    = */ uint32_t,
                                    /* class T_FloatType           = */ float,
                                    /* class T_DoubleType          = */ float,
                                    /* class T_BooleanType         = */ bool,
                                    /* class T_CharType            = */ char
                                    >;
    }
}
#endif // CMD_MESSENGER_HPP
#pragma GCC diagnostic pop
