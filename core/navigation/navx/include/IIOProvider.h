#ifndef SRC_IIOPROVIDER_H_
#define SRC_IIOPROVIDER_H_

#include <stdint.h>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IIOProvider
            {
            public:
                IIOProvider() {}
                virtual bool   isConnected() = 0;
                virtual double getByteCount() = 0;
                virtual double getUpdateCount() = 0;
                virtual void   setUpdateRateHz(uint8_t update_rate) = 0;
                virtual void   zeroYaw() = 0;
                virtual void   zeroDisplacement() = 0;
                virtual void   run() = 0;
                virtual void   stop() = 0;
            };
        }
    }
}
#endif /* SRC_IIOPROVIDER_H_ */
