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
                {

                }
                /*
                Read will supply data to the Object given by Mock 1 unsigned char at a time.
                Write will receive the data sent by the Roboclaw object.
                */
                size_t MockRoboclaw::write(const std::string& message)
                {
                    m_last_sent = message;
                    return message.size();
                }

                size_t MockRoboclaw::write(const std::vector<uint8_t> &data)
                {
                  m_last_cmd = data;
                  return data.size();
                }

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

                std::string MockRoboclaw::read(size_t n)
                {
                    return m_response;
                }
                /*
                size_t
                MockRoboclaw::read(uint8_t *buffer, size_t size)
                {
                    return size;
                }*/

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

                void MockRoboclaw::printResponse()
                {
                  if(m_last_cmd.size() == 0)
                  {
                    std::cout << "vector empty" << std::endl;
                    return;
                  }
                  for (std::vector<uint8_t>::const_iterator i = m_last_cmd.begin(); i != m_last_cmd.end(); i++)
                  {
                    std::cout << std::hex << static_cast<int>(*i) << ' ';
                  }
                }

            }
        }
    }
}
