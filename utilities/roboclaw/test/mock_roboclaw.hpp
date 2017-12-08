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
                class MockRoboclaw :public Roboclaw
                {
                public:

                    //MockRoboclaw();
                    MockRoboclaw(nlohmann::json config={{"address", 0x80},{"timeout", 100 * units::ms},
                    {"ticks_per_rev", 360.0},{"wheel_radius", 4 * units::cm}}, bool test=1);
                    /*
                    wjwoodman serial methods
                    size_t write(const std::vector<uint8_t> &data) override;
                    std::string read(size_t n) override;
                    size_t write(const std::string &message) override;
                    */
                    //pins serial methods
                    void write(std::vector<uint8_t> message) override;
                    std::vector<uint8_t> read(size_t length = 1) override;

                    void setcResponse(const std::vector<uint8_t> response);
                    void setResponse(const std::string& response);
                    uint8_t returnFF() override;
                    std::vector<uint8_t> readN(uint8_t n, Command cmd) override;
                    //size_t read(uint8_t *buffer, size_t size) override;

                    std::vector<uint8_t> getLastCmd();
                    std::string getLastSent();
                    void printResponse();
                    void setBytes(size_t bytes=0xFF);

                private:
                    std::string m_last_sent;
                    std::vector<uint8_t> m_last_cmd;
                    std::vector<uint8_t> m_cresponse;
                    std::string m_response;
                    size_t m_bytes = 0xFF;
                };
            }
        }
    }
}
#endif //MOCK_ROBOCLAW_HPP
