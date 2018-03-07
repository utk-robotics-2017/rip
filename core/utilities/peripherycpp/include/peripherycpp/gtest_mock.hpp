#include "peripherycpp/gpio.hpp"
#include "peripherycpp/i2c.hpp"
#include "peripherycpp/mmio.hpp"
#include "perihperycpp/serial.hpp"
#include "peripherycpp/spi.hpp"
#include "peripherycpp/exceptions.hpp"
#include <gmock/gmock.h>

namespace rip
{

    namespace peripherycpp
    {

        class MockAbsGpio : public AbsGpio 
        {
            public:
                MOCK_METHOD2(open, void(unsigned int pin, int direction));

                MOCK_METHOD0(read, bool());

                MOCK_METHOD1(write, void(bool value));

                MOCK_METHOD1(poll, int(int timeout_ms));

                MOCK_METHOD0(close, void());

                MOCK_METHOD0(supportsInterrupts, bool());

                MOCK_METHOD0(getDirection, int());

                MOCK_METHOD0(getEdge, int());

                MOCK_METHOD1(setDirection, void(int direction));

                MOCK_METHOD1(setEdge, void(int edge));

                MOCK_METHOD0(pin, unsigned int());

                MOCK_METHOD0(fd, int());

                MOCK_METHOD1(toString, std::string(size_t len));
        };

        class MockAbsI2c : public AbsI2c {
            public:
                MOCK_METHOD1(open, void(const std::string path));

                MOCK_METHOD3(transfer, void(std::vector< std::vector<uint8_t> > msg_data, std::vector<int> flags, size_t count));

                MOCK_METHOD0(close, void());

                MOCK_METHOD0(fd, int());

                MOCK_METHOD1(toString, std::string(size_t len));
        };

        class MockAbsSerial : public AbsSerial {
            public:
                MOCK_METHOD2(open, void(std::string device, unsigned int baudrate));

                MOCK_METHOD2(read, std::vector<uint8_t>(size_t len, int timeout_ms));

                MOCK_METHOD3(read, void(uint8_t* buf, size_t size, int timeout_ms));

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

        class MockAbsMmio : public AbsMmio {
            public:
                MOCK_METHOD2(open, void(uintptr_t base, size_t size));

                MOCK_METHOD1(read32, uint32_t(uintptr_t offset));

                MOCK_METHOD1(read16, uint16_t(uintptr_t offset));

                MOCK_METHOD1(read8, uint8_t(uintptr_t offset));

                MOCK_METHOD2(read, std::vector<uint8_t>(uintptr_t offset, size_t len));

                MOCK_METHOD2(write32, void(uintptr_t offset, uint32_t value));

                MOCK_METHOD2(write16, void(uintptr_t offset, uint16_t value));

                MOCK_METHOD2(write8, void(uintptr_t offset, uint8_t value));

                MOCK_METHOD2(write, void(uintptr_t offset, std::vector<uint8_t> &buf));

                MOCK_METHOD0(close, void());

                MOCK_METHOD0(base, uintptr_t());

                MOCK_METHOD0(size, size_t());

                MOCK_METHOD1(toString, std::string(size_t len));
        };

    }

}
