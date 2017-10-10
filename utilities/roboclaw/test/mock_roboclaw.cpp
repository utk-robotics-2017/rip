#include "mock_roboclaw.hpp"

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace mocks
            {
                size_t MockRoboclaw::write(const std::string& message)
                {
                    m_last_sent = message;
                    return message.size();
                }

                std::string MockRoboclaw::getLastSent()
                {
                    return m_last_sent;
                }

                void MockRoboclaw::setResponse(const std::string& response)
                {
                    m_response = response;
                }

                std::string MockRoboclaw::read(size_t n)
                {
                    return m_response;
                }
            }
        }
    }
}
