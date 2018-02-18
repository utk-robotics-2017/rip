#ifndef _PERIPHERY_I2C_HPP
#define _PERIPHERY_I2C_HPP

extern "C"
{
    #include "i2c.h"
}
#include "peripherycpp/exceptions.hpp"
#include <vector>
#include <string>

namespace rip
{

    namespace peripherycpp
    {

        class I2c 
        {
            public:

                /**
                 * open
                 * @param path  the path to the desired i2c-dev device. 
                 * @brief  Open the i2c-dev device at the specified path.
                 */
                void open(const std::string path);

                /**
                 * transfer
                 * @param msg_data  the data for the messages that will be transferred (must be of size count). Each element of the vector will be used for a different message."
                 * @param flags  the flags for the messages that will be trnasferred (must be of size count). Each element of the vector must correspond with the same indexed element of msg_data.
                 * @param count  the number of messages to be transferred.
                 * @brief  Transfer count number of struct i2c_msg I2C messages.
                 */
                void transfer(std::vector< std::vector<uint8_t> > msg_data, std::vector<int> flags, size_t count);

                /**
                 * close
                 * @brief  Close the i2c-dev device.
                 */
                void close();

                /**
                 * fd
                 * @brief  Return the file descriptor (for the underlying i2c-dev device) of the I2C handle.
                 */
                int fd();

                /**
                 * toString
                 * @param len  the size of the string to be returned.
                 * @brief  Return a string representation of the I2C handle.
                 */
                std::string toString(size_t len);

            private:

                i2c_t i2c;

        };

    }

}

#endif
