#include "mock_device.hpp"

namespace rip
{
    namespace utilities
    {
        namespace cmdmessenger
        {
            namespace mocks
            {
                MockDevice::MockDevice()
                    : Device(""),
                      m_correctly_acknowledge(true)
                {
                    setSeparators();
                }

                void MockDevice::setSeparators(char field, char command, char escape)
                {
                    m_field_separator = field;
                    m_command_separator = command;
                    m_escape_character = escape;
                }

                size_t MockDevice::write(const std::string& message)
                {
                    m_last_sent = message;
                    m_response = (m_correctly_acknowledge ? toBytes<int16_t>(0) : toBytes<int16_t>(1)) + m_field_separator + message.substr(0, 2) + m_command_separator;
                    return message.size();
                }

                void MockDevice::setAcknowledgeCorrectly(bool correct)
                {
                    m_correctly_acknowledge = correct;
                }

                std::string MockDevice::getLastSent()
                {
                    return m_last_sent;
                }

                void MockDevice::setResponse(const std::string& response)
                {
                    m_response = response;
                }

                std::string MockDevice::readline(size_t size, std::string eol)
                {
                    std::string response = m_response;
                    m_response.clear();
                    return response;
                }

            }
        }
    }
}
