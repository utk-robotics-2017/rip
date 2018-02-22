/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef PERIPHERY_EXCEPTIONS_H
#define PERIPHERY_EXCEPTIONS_H
#include <misc/exception_base.hpp>
namespace rip
{
    namespace peripherycpp
    {
        /**
         * @class SerialArgError
         * @brief Exception for when invalid serial arguments.
         */
        NEW_EX(SerialArgError)

        /**
         * @class SerialOpenError
         * @brief Exception for when serial fails to open.
         */
        NEW_EX(SerialOpenError)

        /**
         * @class SerialQueryError
         * @brief Exception when serial encounters an IO failure cannot query port attributes.
         */
        NEW_EX(SerialQueryError)

        /**
         * @class SerialIoError
         * @brief Exception when serial encounters an IO failure.
         */
        NEW_EX(SerialIoError)

        /**
         * @class SerialConfigureError
         * @brief Exception when serial fails to configure.
         */
        NEW_EX(SerialConfigureError)

        /**
         * @class SerialCloseError
         * @brief Exception when serial fails to close.
         */
        NEW_EX(SerialCloseError)

        /**
         * @class I2cArgError
         * @brief Excpetion for invalid I2C arguments.
         */
        NEW_EX(I2cArgError)

        /**
         * @class I2cOpenError
         * @brief Exception for an error when opening I2C device.
         */
        NEW_EX(I2cOpenError)

        /**
         * @class I2cQuerySupportError
         * @brief Exception for an error occured while querying I2C support on I2C device.
         */
        NEW_EX(I2cQuerySupportError)

        /**
         * @class I2cNotSupportedError
         * @brief Exception for when I2C is not supported on the device.
         */
        NEW_EX(I2cNotSupportedError)

        /**
         * @class I2cTransferError
         * @brief Exeption for an error during I2C transfer.
         */
        NEW_EX(I2cTransferError)

        /**
         * @class I2cCloseError
         * @brief Exception for an error when closing the I2C device.
         */
        NEW_EX(I2cCloseError)

        /**
         * @class GpioArgError
         * @brief Exception for invalid GPIO arguments.
         */
        NEW_EX(GpioArgError)

        /**
         * @class GpioExportError
         * @brief Exception for an error during GPIO export.
         */
        NEW_EX(GpioExportError)

        /**
         * @class GpioOpenError
         * @brief Exception for an error during opening the GPIO value.
         */
        NEW_EX(GpioOpenError)

        /**
         * @class GpioIoError
         * @brief Exception for an error during reading/writing the GPIO value.
         */
        NEW_EX(GpioIoError)

        /**
         * @class GpioCloseError
         * @brief Exception for an error when closing the GPIO value.
         */
        NEW_EX(GpioCloseError)

        /**
         * @class GpioSetDirectionError
         * @brief Exception for an error when setting the GPIO direction.
         */
        NEW_EX(GpioSetDirectionError)

        /**
         * @class GpioGetDirectionError
         * @brief Exception for an error when getting the GPIO direction.
         */
        NEW_EX(GpioGetDirectionError)

        /**
         * @class GpioSetEdgeError
         * @brief Exception for an error when setting the GPIO interrupt edge.
         */
        NEW_EX(GpioSetEdgeError)

        /**
         * @class GpioGetEdgeError
         * @brief Exception for an error when getting the GPIO interrupt edge.
         */
        NEW_EX(GpioGetEdgeError)

        /**
         * @class MmioArgError
         * @brief Exception for invalid MMIO arguments.
         */
        NEW_EX(MmioArgError)

        /**
         * @class MmioOpenError
         * @brief Exception for error when opening /dev/mem.
         */
        NEW_EX(MmioOpenError)

        /**
         * @class MmioMapError
         * @brief Exception for an error when mapping memory.
         */
        NEW_EX(MmioMapError)

        /**
         * @class MmioCloseError
         * @brief Exception for an error when closing /dev/mem.
         */
        NEW_EX(MmioCloseError)

        /**
         * @class MmioUnmapError
         * @brief Exception for an error when unmapping memory.
         */
        NEW_EX(MmioUnmapError)

        /**
         * @class SpiArgError
         * @brief Exception for invalid SPI arguements.
         */
        NEW_EX(SpiArgError)

        /**
         * @class SpiOpenError
         * @brief Exception for an error when opening SPI device.
         */
        NEW_EX(SpiOpenError)

        /**
         * @class SpiQueryError
         * @brief Exception for an error when querying SPI device settings.
         */
        NEW_EX(SpiQueryError)

        /**
         * @class SpiConfigureError
         * @brief Exception for an error when configuring SPI device.
         */
        NEW_EX(SpiConfigureError)

        /**
         * @class SpiTransferError
         * @brief Exception for an error during a SPI transfer.
         */
        NEW_EX(SpiTransferError)

        /**
         * @class SpiCloseError
         * @brief Exception for an error when closing SPI device.
         */
        NEW_EX(SpiCloseError)
    }
}

#endif //PERIPHERY_EXCEPTIONS_H
