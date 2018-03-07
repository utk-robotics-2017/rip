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

        class AbsI2c
        {
            public:
                virtual ~I2c() {}

                //I2c(const std::string path);

                virtual void open(const std::string path) = 0;

                virtual void transfer(std::vector< std::vector<uint8_t> > msg_data, std::vector<int> flags, size_t count) = 0;

                virtual void close() = 0;

                virtual int fd() = 0;

                virtual std::string toString(size_t len) = 0;

        };

        class I2c : public AbsI2c
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
