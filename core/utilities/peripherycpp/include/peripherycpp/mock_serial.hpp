#include "peripherycpp/serial.hpp"
#include <gmock/gmock.h>

namespace rip
{

    namespace peripherycpp
    {

        namespace mock
        {

            class MockAbsSerial : public AbsSerial {
                public:
                    MOCK_METHOD2(open, void(std::string device, unsigned int baudrate));

                    MOCK_METHOD2(read, std::vector<uint8_t>(size_t len, int timeout_ms));

                    MOCK_METHOD3(read, void(uint8_t* buf, size_t size, int timeout_ms));

                    MOCK_METHOD3(read, void(char* buf, size_t size, int timeout_ms));

                    MOCK_METHOD2(write, void(uint8_t* data, size_t size));

                    MOCK_METHOD2(write, void(char* data, size_t size));

                    MOCK_METHOD1(write, void(std::vector<uint8_t> data));
                
                    MOCK_METHOD0(flush, void());

                    MOCK_METHOD0(outputWaiting, unsigned int());

                    MOCK_METHOD0(inputWaiting, unsigned int());
                
                    MOCK_METHOD1(poll, bool(int timeout_ms));

                    MOCK_METHOD0(close, void());

                    MOCK_METHOD0(getBaudrate, uint32_t());

                    MOCK_METHOD0(getDatabits, unsigned int());

                    MOCK_METHOD0(getParity, int());

                    MOCK_METHOD0(getStopbits, unsigned int());

                    MOCK_METHOD0(getxOnxOff, bool());

                    MOCK_METHOD0(getRtscts, bool());

                    MOCK_METHOD1(setBaudrate, void(uint32_t baudrate));
                
                    MOCK_METHOD1(setDatabits, void(unsigned int databits));

                    MOCK_METHOD1(setParity, void(int parity));

                    MOCK_METHOD1(setStopBits, void(unsigned int stopbits));

                    MOCK_METHOD1(setxOnxOff, void(bool enabled));

                    MOCK_METHOD1(setRtscts, void(bool enabled));
            };

        }

    }

}
