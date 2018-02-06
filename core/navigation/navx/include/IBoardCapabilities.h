#ifndef SRC_IBOARDCAPABILITIES_H_
#define SRC_IBOARDCAPABILITIES_H_

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            struct IBoardCapabilities
            {
                IBoardCapabilities() {}
                virtual bool isOmniMountSupported() = 0;
                virtual bool isBoardYawResetSupported() = 0;
                virtual bool isDisplacementSupported() = 0;
                virtual bool isAHRSPosTimestampSupported() = 0;
            };
        }
    }
}

#endif /* SRC_IBOARDCAPABILITIES_H_ */
