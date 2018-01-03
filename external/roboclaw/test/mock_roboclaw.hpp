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
                class MockRoboclaw
                {
                public:
                    size_t MockRoboclaw::write(const std::string& message) override;

                    std::string MockRoboclaw::getLastSent();

                    void MockRoboclaw::setResponse(const std::string& response);

                    std::string MockRoboclaw::read(size_t n) override;
                };
            }
        }
    }
}
#endif //MOCK_ROBOCLAW_HPP
