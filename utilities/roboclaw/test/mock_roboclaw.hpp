#ifndef MOCK_ROBOCLAW_HPP
#define MOCK_ROBOCLAW_HPP

#include "roboclaw.hpp"

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace mocks
            {
                class MockRoboclaw : public Roboclaw
                {
                public:
                    size_t write(const std::string& message);

                    std::string getLastSent();

                    void setResponse(const std::string& response);

                    std::string read(size_t n);

                private:
                    std::string m_last_sent;
                    std::string m_response;
                };
            }
        }
    }
}
#endif //MOCK_ROBOCLAW_HPP
