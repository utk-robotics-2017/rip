#include "cmd_messenger/device.hpp"

#include "cmd_messenger/exceptions.hpp"

#include <string>
#include <iostream>

namespace rip
{

    namespace cmdmessenger
    {
        namespace mocks
        {
            class MockDevice : public Device
            {
            public:
                MockDevice();

                void open() override
                {

                }

                bool isOpen () const override
                {
                  return true;
                }

                void flush() override
                {

                }

                void flushInput() override
                {

                }

                void flushOutput() override
                {

                }

                std::string read(size_t size) override
                {
                    std::string rv;
                    rv = m_response.substr(0, size);
                    //std::cout << "MockDevice Read(" << size << "): \"" << rv << "\"" << std::endl;
                    m_response.erase(0, size);
                    return rv;
                }

                /**
                 * @brief write
                 * @param message
                 * @return
                 */
                size_t write(const std::string& message) override;

                /**
                 * @brief setAcknowledgeCorrectly
                 * @param correct
                 */
                void setAcknowledgeCorrectly(bool correct);

                /**
                 * @brief readline
                 * @param size
                 * @param eol
                 * @return
                 */
                std::string readline(size_t size = 65536, std::string eol = "\n") override;

                /**
                 * @brief getLastSent
                 * @return
                 */
                std::string getLastSent();

                /**
                 * @brief setResponse
                 * @param response
                 */
                void setResponse(const std::string& response);

                /**
                 * @brief getResponse
                 * @return current response_tuple
                 */
                std::string getResponse();

                /**
                 * @brief setSeparators
                 * @param field
                 * @param command
                 * @param escape
                 */
                void setSeparators(char field = ',', char command = ';', char escape = '/');

                /**
                 * @brief Convert a type into bytes
                 *
                 * @param t The value to convert into bytes
                 *
                 * @returns A string (as a list of chars)
                 */
                template<typename T>
                std::string toBytes(const T& t)
                {
                    char* byte_pointer = reinterpret_cast<char*>(const_cast<T*>(&t));
                    std::string rv;
                    for (size_t i = 0; i < sizeof(t); i++)
                    {
                        // Add the escape character
                        if (*byte_pointer == m_field_separator ||
                                *byte_pointer == m_command_separator ||
                                *byte_pointer == m_escape_character)
                        {
                            rv.push_back(m_escape_character);
                        }
                        rv.push_back(*byte_pointer);
                        byte_pointer ++;
                    }
                    return rv;
                }

                /**
                 * @brief Pull a string out of the command
                 *
                 * @param message A string as a representation of bytes
                 *
                 * @note This version is used when the string can convert directly to the type
                 */
                template<typename T>
                typename std::enable_if<std::is_convertible<std::string, T>::value, T>::type fromBytesString(std::string& message)
                {
                    std::string rv;
                    int comma_position = message.find("" + m_escape_character + m_field_separator);
                    if (static_cast<size_t>(comma_position) != std::string::npos)
                    {
                        rv = message.substr(0, comma_position);
                        message.erase(0, comma_position);
                        return rv;
                    }
                    return message.substr(0, message.size() - 1); // remove command separator
                }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
                /**
                 * @brief The version of the fromBytesString
                 *
                 * @param message A string as a representation of bytes
                 *
                 * @note This version is used when the string cannot convert directly to the type
                 *
                 * @exception UnconvertibleArgument Thrown because conversion is not possible
                 */
                template<typename T>
                typename std::enable_if<not std::is_convertible<std::string, T>::value, T>::type fromBytesString(std::string& message)
                {
                    throw cmdmessenger::UnconvertibleArgument();
                    return T();
                }
#pragma GCC diagnostic pop

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
                    if (std::is_same<T, std::string>::value)
                    {
                        return fromBytesString<T>(message);
                    }
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
                    message.erase(0, sizeof(rv) + escape_chars_skipped);
                    return rv;

                }

            private:
                std::string m_response;
                std::string m_last_sent;
                bool m_correctly_acknowledge;
                bool m_self_is_open;
                char m_field_separator;
                char m_command_separator;
                char m_escape_character;
            };
        }
    }
}
