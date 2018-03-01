/*
 * IRegisterIO.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott, UTK IEEE Robotics
 */

#ifndef SRC_IREGISTERIO_H_
#define SRC_IREGISTERIO_H_

#include <stdint.h>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IRegisterIO
            {
            public:
                IRegisterIO(){}
                virtual bool init() = 0;
                virtual bool write(uint8_t address, uint8_t value ) = 0;
                virtual bool read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len) = 0;
                virtual bool shutdown() = 0;
                virtual void enableLogging(bool enable) = 0;
            };
        }
    }
}

#endif /* SRC_IREGISTERIO_H_ */
