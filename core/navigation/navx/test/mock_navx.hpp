#ifndef MOCK_NAVX_HPP
#define MOCK_NAVX_HPP
#include <AHRS.h>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace mocks
            {
                class MockNavX : AHRS
                {
                public:
                    //void setResponse();
                private:
                    //m_response;
                }
            }
        }
    }
}
#endif //MOCK_NAVX_HPP
