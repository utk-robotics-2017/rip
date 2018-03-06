/*
 * RegisterIOI2C.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott, IEEE Robotics UTK
 */

#ifndef SRC_REGISTERIOI2C_H_
#define SRC_REGISTERIOI2C_H_

#include <misc/logger.hpp>
#include <fmt/format.h>
#include "i_register_io.hpp"
#include <peripherycpp/i2c.hpp>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class RegisterIO_I2C : public IRegisterIO
            {
            public:
                RegisterIO_I2C(peripherycpp::I2c *port);
                virtual ~RegisterIO_I2C() {}
                bool init();
                bool write(uint8_t address, uint8_t value);
                bool read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len);
                bool shutdown();
                void enableLogging(bool enable);
            };
        }
    }
}

#endif /* SRC_REGISTERIOI2C_H_ */
