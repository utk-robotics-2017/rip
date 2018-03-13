#include <string>
#include <vector>
#include "peripherycpp/i2c.hpp"
#include "peripherycpp/exceptions.hpp"

#define EEPROM_I2C_ADDR 0x50

namespace rip
{
    namespace peripherycpp
    {
        I2c::I2c()
        {}

        I2c::I2c(const std::string path)
        {
            open(path);
        }

        void I2c::open(const std::string path)
        {
            checkError(i2c_open(&m_i2c, path.c_str()));
        }

        uint8_t I2c::read_byte(uint8_t addr)
        {

        }

        std::vector<uint8_t> I2c::read(uint8_t addr, size_t len)
        {

        }

        void I2c::write_byte(uint8_t addr, uint8_t byte)
        {
            struct i2c_msg *msg = new struct i2c_msg;
            uint8_t buf[2] = {addr, byte};
            *msg = { .addr = EEPROM_I2C_ADDR, .flags = 0, .len = 1, .buf = buf };
            int err = i2c_transfer(&m_i2c, msg, 1);
            delete [] msg;
            checkError(err);
        }

        void I2c::write(uint8_t addr, const std::vector<uint8_t>& data)
        {

        }

        void I2c::transfer(std::vector< std::vector<uint8_t> > msg_data, std::vector<int> flags, size_t count)
        {
            //first
            /* Notes about this function:
             * 1) `msg_data` stores the data that will be transfered. It must be a vector of
             *        vectors of uint8_t's to allow for determining the amount of data being
             *        transfered.
             * 2) `flags` stores integers with values of either 0 or 1 (can actually be any
             *        number besides 0). A value of 0 means that the transfer will be a write,
             *        and a value of 1 means that the transfer will be a read.
             * 3) The element at index i of `msg_data` must correspond with the element at index
             *        i of `flags`. The transfer will not work correctly if this is not done.
             * 4) `count` is an int (technically size_t) that states how many reads or writes will
             *        be performed during this function call. Both `msg_data` and `flags` must have
             *        a size equal to `count`.
             */
            if (msg_data.size() != count || flags.size() != count)
            {
                throw I2cArgError("msg_data and flags must be the same size.");
            }
            struct i2c_msg *msgs = new struct i2c_msg[count];
            uint8_t *data;
            uint16_t size;
            for (int i = 0; i < (int)count; i++)
            {
                data = &(msg_data[i][0]);
                size = msg_data[i].size();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
                if (flags[i] == 0)
                {
                    msgs[i] = { .addr = EEPROM_I2C_ADDR, .flags = 0, .len = size, .buf = data };
                }
                else
                {
                    msgs[i] = { .addr = EEPROM_I2C_ADDR, .flags = I2C_M_RD, .len = size, .buf = data };
                }
#pragma GCC diagnostic pop
            }
            int err = i2c_transfer(&m_i2c, msgs, count);
            delete [] msgs;
            checkError(err);
        }

        void I2c::close()
        {
            checkError(i2c_close(&m_i2c));
        }

        int I2c::fd()
        {
            int fd = i2c_fd(&m_i2c);
            return fd;
        }

        std::string I2c::toString(size_t len)
        {
            char *str = new char[len];
            i2c_tostring(&m_i2c, str, len);
            std::string ret(str);
            delete [] str;
            return ret;
        }

        void I2c::checkError(int err_code)
        {
            switch(err_code)
            {
                case I2C_ERROR_ARG: throw I2cArgError(i2c_errmsg(&m_i2c)); break;
                case I2C_ERROR_OPEN: throw I2cOpenError(i2c_errmsg(&m_i2c)); break;
                case I2C_ERROR_QUERY_SUPPORT: throw I2cQuerySupportError(i2c_errmsg(&m_i2c)); break;
                case I2C_ERROR_NOT_SUPPORTED: throw I2cNotSupportedError(i2c_errmsg(&m_i2c)); break;
                case I2C_ERROR_TRANSFER: throw I2cTransferError(i2c_errmsg(&m_i2c)); break;
                case I2C_ERROR_CLOSE: throw I2cCloseError(i2c_errmsg(&m_i2c)); break;
                default: break;
            }
        }
    }

}
