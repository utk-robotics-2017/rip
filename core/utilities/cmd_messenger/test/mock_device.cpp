#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
// Totally acceptable for a mock to have unused parameters

#include "mock_device.hpp"

namespace rip
{
    namespace cmdmessenger
    {
        namespace mocks
        {
            MockDevice::MockDevice()
                : Device(""),
                  m_correctly_acknowledge(true),
                  m_response(""),
                  m_last_sent("")
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
                m_response = std::string(m_correctly_acknowledge ? "0" : "1") + m_field_separator + toBytes<uint16_t>(std::stoi(message.substr(0, message.find(m_field_separator)))) + m_command_separator;
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

            std::string MockDevice::getResponse()
            {
              return m_response;
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
#pragma GCC diagnostic pop
