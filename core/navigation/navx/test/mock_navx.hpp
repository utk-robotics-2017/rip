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
                class MockNavX : NavX
                {
                public:
                    //void setResponse();
                    void int read(char *data, int size) override;

                    void write(char *data, int length) override;


                private:
                    //m_response;
                }
            }
        }
    }
}
#endif //MOCK_NAVX_HPP
