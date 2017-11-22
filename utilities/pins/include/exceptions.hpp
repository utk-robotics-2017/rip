#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP
#include <exception_base.hpp>

namespace rip
{
    namespace utilities
    {
        namespace pins
        {

            /* @class FileAccessError
             * @brief Used for when a file cannot be accessed (ex. permissions or existence)
             */
            NEW_EX(FileAccessError);

            /* @class DigitalReadError
             * @brief Used when we don't get a proper read from a digital input (ex. not 0|1)
             */
            NEW_EX(DigitalReadError);

            namespace i2c
            {
                /* @class I2CWriteError
                 * @brief Used when the I2C write() or read() returns an error code
                 */
                NEW_EX(I2CWriteError);

                /* @class I2CWriteError
                 * @brief Used when the I2C write() or read() returns an error code
                 */
                NEW_EX(I2CReadError);

                /* @class SlaveAddressNotSet
                 * @brief If the I2C slave address isn't set; you gotta set it lol
                 */
                NEW_EX(SlaveAddressNotSet);

                /* @class BadAddressError
                 * @brief For when the I2C address is incorrect, improper, badly formatted
                 */
                NEW_EX(BadAddressError)
            }
        } // pins
    } // utilities
} // rip

#endif // EXCEPTIONS_HPP
