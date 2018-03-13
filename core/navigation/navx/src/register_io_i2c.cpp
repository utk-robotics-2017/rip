/*
 * RegisterIOI2C.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott, IEEE robotics
 */

#include <navx/register_io_i2c.hpp>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {

            RegisterIO_I2C::RegisterIO_I2C(peripherycpp::I2c* port)
            {
                this->port = port;
                this->trace = false;
            }

            bool RegisterIO_I2C::init()
            {
                return true;
            }

            bool RegisterIO_I2C::write(uint8_t address, uint8_t value )
            {
            	bool aborted = port->write(address | 0x80, value);
                //if (aborted && trace) printf("navX-MXP I2C Write error\n");
                return !aborted;
            }

            static int MAX_WPILIB_I2C_READ_BYTES = 127;

            bool RegisterIO_I2C::read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len)
            {
            	int len = buffer_len;
                int buffer_offset = 0;
                uint8_t read_buffer[MAX_WPILIB_I2C_READ_BYTES];
                while (len > 0)
                {
                    int read_len = (len > MAX_WPILIB_I2C_READ_BYTES) ? MAX_WPILIB_I2C_READ_BYTES : len;
                    if (!port->write(first_address + buffer_offset, read_len) &&
                        !port->readOnly(read_len, read_buffer))
                        {
                        memcpy(buffer + buffer_offset, read_buffer, read_len);
                        buffer_offset += read_len;
                        len -= read_len;
                    }
                    else
                    {
                        //read error
                        break;
                    }
                }
                return (len == 0);
            }

            bool RegisterIO_I2C::shutdown()
            {
                return true;
            }

            void RegisterIO_I2C::enableLogging(bool enable)
            {
            	trace = enable;
            }

        }
    }
}
