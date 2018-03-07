#include "peripherycpp/mmio.hpp"
#include <gmock/gmock.h>

namespace rip
{

    namespace peripherycpp
    {

        namespace mock
        {

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

}
