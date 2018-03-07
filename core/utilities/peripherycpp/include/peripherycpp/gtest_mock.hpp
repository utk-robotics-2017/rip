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

        
    }

}
