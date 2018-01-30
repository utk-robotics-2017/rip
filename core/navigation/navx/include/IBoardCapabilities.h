#ifndef SRC_IBOARDCAPABILITIES_H_
#define SRC_IBOARDCAPABILITIES_H_

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IBoardCapabilities {
            public:
                IBoardCapabilities() {}
                virtual bool IsOmniMountSupported() = 0;
                virtual bool IsBoardYawResetSupported() = 0;
                virtual bool IsDisplacementSupported() = 0;
                virtual bool IsAHRSPosTimestampSupported() = 0;
            };
        }
    }
}

#endif /* SRC_IBOARDCAPABILITIES_H_ */
