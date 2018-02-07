#include "cmd_messenger/device.hpp"

#include "cmd_messenger/exceptions.hpp"

#include <string>

namespace rip
{
    namespace utilities
    {
        namespace cmdmessenger
        {
            namespace mocks
            {
                class MockDevice : public Device
                {
                public:
                    MockDevice();

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
                     * @brief setSeparators
                     * @param field
                     * @param command
                     * @param escape
                     */
                    void setSeparators(char field = ',', char command = ';', char escape = '\\');

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
                        for (int i = 0; i < sizeof(t); i++)
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
                        if (comma_position != std::string::npos)
                        {
                            rv = message.substr(0, comma_position);
                            message.erase(0, comma_position);
                            return rv;
                        }
                        return message.substr(0, message.size() - 1); // remove command separator
                    }

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
                        for (int i = 0; i < sizeof(rv); i++)
                        {
                            // Skip the escape character
                            if (message[i] == m_escape_character)
                            {
                                i++;
                            }
                            *byte_pointer = message[i];
                            byte_pointer ++;
                        }
                        message.erase(0, sizeof(rv));
                        return rv;

                    }
                private:
                    std::string m_response;
                    std::string m_last_sent;
                    bool m_correctly_acknowledge;
                    char m_field_separator;
                    char m_command_separator;
                    char m_escape_character;
                };
            }
        }
    }
}
