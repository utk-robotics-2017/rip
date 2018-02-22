#ifndef FAKE_NAVX_HPP
#define FAKE_NAVX_HPP
#include <navx/navx.hpp>
#include <vector>
#include <stdint.h>
#include <navx/serial_port.hpp>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            namespace fakes
            {
                class FakeNavX : SerialPort
                {
                public:
                    /**
                     * overrides SerialIO read function.
                     * @param  data raw data
                     * @param  number of elements
                     * @return int?
                     */
                    int read(char *data, int size) override;
                    /**
                     * [getWritten description]
                     * @return [description]
                     */
                    const std::vector<char>* getWritten();
                    /**
                     * [write description]
                     * @param data   [description]
                     * @param length [description]
                     */
                    void write(char *data, int length) override;

                    /**
                     * [setResponse description]
                     * @param data [description]
                     */
                    void setResponse(std::vector<char> data);
                private:
                    std::vector<char> m_response;
                    std::vector<char> m_written;
                };
            }
        }
    }
}

#endif //FAKE_NAVX_HPP
