#include "mock_roboclaw.hpp"

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace mocks
            {
                MockRoboclaw::MockRoboclaw(nlohmann::json config, bool test) :Roboclaw(config, test)
                {}

                std::string MockRoboclaw::getLastSent()
                {
                    return m_last_sent;
                }

                void MockRoboclaw::setcResponse(const std::vector<uint8_t> response)
                {
                    m_cresponse = response;
                }

                std::vector<uint8_t> MockRoboclaw::readN(uint8_t n, Command cmd)
                {
                    return m_cresponse;
                }

                void MockRoboclaw::write(serial_t *m_serial, std::vector<uint8_t> command, size_t len)
                {
                    m_last_cmd = command;
                }
                uint8_t MockRoboclaw::read(serial_t *serial, units::Time timeout_ms)
                {
                    return 0;
                }

                void MockRoboclaw::setBytes(size_t bytes)
                {
                    m_bytes = bytes;
                }

                uint8_t MockRoboclaw::returnFF()
                {
                  return m_bytes;
                }

                std::vector<uint8_t> MockRoboclaw::getLastCmd()
                {
                  return m_last_cmd;
                }

                //debugging tool for me, will remove eventually
                void MockRoboclaw::printResponse()
                {
                  if(m_last_cmd.size() == 0)
                  {
                    // std::cout << "vector empty" << std::endl;
                    return;
                  }
                  for (std::vector<uint8_t>::const_iterator i = m_last_cmd.begin(); i != m_last_cmd.end(); i++)
                  {
                    // std::cout << std::hex << static_cast<int>(*i) << ' ';
                  }
                }

            }
        }
    }
}
