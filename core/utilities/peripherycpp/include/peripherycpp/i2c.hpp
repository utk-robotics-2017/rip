#ifndef _PERIPHERY_I2C_HPP
#define _PERIPHERY_I2C_HPP

extern "C"
{
    #include "i2c.h"
}
#include <peripherycpp/exceptions.hpp>
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
                 * default constructor
                 */
                I2c();
                /**
                 * constructor that opens given path
                 * @param path the path to the desired i2c-dev device.
                 */
                I2c(const std::string path);
                /**
                 * open
                 * @param path  the path to the desired i2c-dev device.
                 * @brief  Open the i2c-dev device at the specified path.
                 */
                void open(const std::string path);
                /**
                 * Simplified read method
                 * @param  addr i2c address to read from
                 * @param  len  # of bytes to read
                 * @return      vector of bytes
                 */
                std::vector<uint8_t> read(uint8_t addr, size_t len);
                /**
                 * reads one byte from a given i2c address
                 * @param  addr address of device to read from
                 * @return      a byte
                 */
                uint8_t read_byte(uint8_t addr);
                /**
                 * Writes a vector of bytes to an i2c device
                 * @param addr address of i2c device
                 * @param data const by reference vector of bytes to write
                 */
                void write(uint8_t addr, const std::vector<uint8_t>& data);
                /**
                 * Writes a single byte to a given i2c address
                 * @param addr address to write to
                 * @param byte byte to write
                 */
                void write_byte(uint8_t addr, uint8_t byte)

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
                 * @return  the file descriptor of the I2C handle.
                 */
                int fd();

                /**
                 * toString
                 * @param len  the size of the string to be returned.
                 * @brief  Return a string representation of the I2C handle.
                 * @return  the string representation of the I2C handle.
                 */
                std::string toString(size_t len);

            private:

                /**
                 * checkError
                 * @param err_code An int error code from Periphery I2C
                 * @brief Acts as a error handler for the I2C class
                 */
                void checkError(int err_code);

                i2c_t m_i2c;

        };

    }

}

#endif
