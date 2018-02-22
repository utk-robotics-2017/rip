#include "fake_navx.hpp"
#include <navx/navx.hpp>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace fakes
            {
                void FakeNavX::write(char *data, int length)
                {
                    for(int i=0; i<length; i++)
                    {
                        m_written.push_back(data[i]);
                    }
                }

                int FakeNavX::read(char *data, int size)
                {
                    m_response.resize(size);
                    m_response.shrink_to_fit();
                    data = &m_response[0];
                    return 0;
                }

                void FakeNavX::setResponse(std::vector<char> data)
                {
                    m_response = data;
                }

                const std::vector<char>* FakeNavX::getWritten()
                {
                    return &m_response;
                }
            }
        }
    }
}
