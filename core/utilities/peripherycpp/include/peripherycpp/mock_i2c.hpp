#include "peripherycpp/i2c.cpp"
#include <gmock/gmock.h>

namespace rip
{

    namespace peripherycpp
    {

        namespace mock
        {

            class MockAbsI2c : public AbsI2c {
                public:
                    MOCK_METHOD1(open, void(const std::string path));

                    MOCK_METHOD3(transfer, void(std::vector< std::vector<uint8_t> > msg_data, std::vector<int> flags, size_t count));

                    MOCK_METHOD0(close, void());

                    MOCK_METHOD0(fd, int());

                    MOCK_METHOD1(toString, std::string(size_t len));
            };

        }

    }

}
