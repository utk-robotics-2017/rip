#ifndef MOCK_NAVX_HPP
#define MOCK_NAVX_HPP
#include <navx/navx.h>
#include <vector>
#include <stdint.h>
#include <navx/SerialPort.h>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace mocks
            {
                class MockNavX : SerialPort
                {
                public:
                    /**
                     * sets the data to be sent to navX code
                     * @param response raw byte data to be passed
                     */
                    void setResponse(char *response);
                    /**
                     * overrides SerialIO read function.
                     * @param  data raw data
                     * @param  number of elements
                     * @return int?
                     */
                    int read(char *data, int size) override;
                    /**
                     * last response
                     * @return [description]
                     */
                    std::vector<uint8_t> getLastResponse();
                    void write(char *data, int length) override;

                private:
                    std::vector<uint8_t> m_response;
                    std::vector<uint8_t> m_lastResponse;
                };
            }
        }
    }
}

#endif //MOCK_NAVX_HPP
