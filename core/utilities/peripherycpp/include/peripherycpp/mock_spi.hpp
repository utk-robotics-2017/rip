#include "peripherycpp/spi.hpp"
#include <gmock/gmock.h>

namespace rip
{

    namespace peripherycpp
    {

        namespace mock
        {
            class MockAbsSpi : public AbsSpi {
                public:
                    MOCK_METHOD3(open, void(const std::string path, unsigned int mode, uint32_t max_speed));

                    MOCK_METHOD6(openAdvanced, void(const std::string path, unsigned int mode, uint32_t max_speed, int bit_order, uint8_t bits_per_word, uint8_t extra_flags));

                    MOCK_METHOD3(transfer, void(const uint8_t *txbuf, uint8_t *rxbuf, size_t len));

                    MOCK_METHOD0(close, void());

                    MOCK_METHOD0(getMode, unsigned int());

                    MOCK_METHOD0(getMaxSpeed, uint32_t());

                    MOCK_METHOD0(getBitOrder, int());

                    MOCK_METHOD0(getBitsPerWord, uint8_t());

                    MOCK_METHOD0(getExtraFlags, uint8_t());

                    MOCK_METHOD1(setMode, void(unsigned int mode));

                    MOCK_METHOD1(setMaxSpeed, void(uint32_t max_speed));

                    MOCK_METHOD1(setBitOrder, void(int bit_order));

                    MOCK_METHOD1(setBitsPerWord, void(uint8_t bits_per_word));

                    MOCK_METHOD1(setExtraFlags, void(uint8_t extra_flags));

                    MOCK_METHOD0(fd, int());

                    MOCK_METHOD1(toString, std::string(size_t len));
            };

        }

    }

}
