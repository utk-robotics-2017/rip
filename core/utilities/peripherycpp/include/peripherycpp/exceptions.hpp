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
#include <exception_base.hpp>
namespace rip
{
    namespace peripherycpp
    {
            /**
             * @class ExampleName
             * @brief Brief description of exception.
             */
            NEW_EX(ExampleName);
            /**
             * @class SerialOpenFail
             * @param SerialOpenFail Serial fails to open.
             */
            NEW_EX(SerialOpenFail);
            /**
             * @class SerialReadFailure
             * @param SerialReadFailure Failure to read bytes.
             */
            NEW_EX(SerialReadFailure);
            /**
             * @class SerialWriteFailure
             * @param SerialWriteFailure Serial writing fails.
             */
            NEW_EX(SerialWriteFailure);
            /**
             * @class SerialFlushFailure
             * @param SerialFlushFailure Serial fails to flush.
             */
            NEW_EX(SerialFlushFailure);
            /**
             * @class SerialCloseFailure
             * @param SerialCloseFailure Serial fails to close.
             */
            NEW_EX(SerialCloseFailure);
            /**
             * @class SerialGetFailure
             * @param SerialGetFailure Failed to get data from serial struct.
             */
            NEW_EX(SerialGetFailure);
            /**
             * @class SerialSetFailure
             * @param SerialSetFailure Failed to set data in serial struct.
             */
            NEW_EX(SerialSetFailure);
            /**
             * @class I2cArgError
             * @param I2cArgError Invalid arguments.
             */
            NEW_EX(I2cArgError);
            /**
             * @class I2cOpenError
             * @param I2cOpenError Error occured while opening I2C device.
             */
            NEW_EX(I2cOpenError);
            /**
             * @class I2cQuerySupportError
             * @param I2cQuerySupportError Error occured while querying I2C support on I2C device.
             */
            NEW_EX(I2cQuerySupportError);
            /**
             * @class I2cNotSupportedError
             * @param I2cNotSupportedError I2C not supported on this device.
             */
            NEW_EX(I2cNotSupportedError);
            /**
             * @class I2cTransferError
             * @param I2cTransferError Error occured during I2C transfer.
             */
            NEW_EX(I2cTransferError);
            /**
             * @class I2cCloseError
             * @param I2cCloseError Error occured while closing the I2C device.
             */
            NEW_EX(I2cCloseError);
            /**
             * @class GpioArgError
             * @param GpioArgError Invalid arguments.
             */
            NEW_EX(GpioArgError);
            /**
             * @class GpioExportError
             * @param GpioExportError Error occured during GPIO export.
             */
            NEW_EX(GpioExportError);
            /**
             * @class GpioOpenError
             * @param GpioOpenError Error occured while opening the GPIO value.
             */
            NEW_EX(GpioOpenError);
            /**
             * @class GpioIoError
             * @param GpioIoError Error occured while reading/writing the GPIO value.
             */
            NEW_EX(GpioIoError);
            /**
             * @class GpioCloseError
             * @param GpioCloseError Error occured while closing the GPIO value.
             */
            NEW_EX(GpioCloseError);
            /**
             * @class GpioSetDirectionError
             * @param GpioSetDirectionError Error occured while setting the GPIO direction.
             */
            NEW_EX(GpioSetDirectionError);
            /**
             * @class GpioGetDirectionError
             * @param GpioGetDirectionError Error occured while getting the GPIO direction.
             */
            NEW_EX(GpioGetDirectionError);
            /**
             * @class GpioSetEdgeError
             * @param GpioSetEdgeError Error occured while setting the GPIO interrupt edge.
             */
            NEW_EX(GpioSetEdgeError);
            /**
             * @class GpioGetEdgeError
             * @param GpioGetEdgeError Error occured while getting the GPIO interrupt edge.
             */
            NEW_EX(GpioGetEdgeError);
            /**
             * @class MmioArgError
             * @param MmioArgError Invalid arguments.
             */
            NEW_EX(MmioArgError);
            /**
             * @class MmioOpenError
             * @param MmioOpenError Error occured while opening /dev/mem.
             */
            NEW_EX(MmioOpenError);
            /**
             * @class MmioMapError
             * @param MmioMapError Error occured while mapping memory.
             */
            NEW_EX(MmioMapError);
            /**
             * @class MmioCloseError
             * @param MmioCloseError Error occured while closing /dev/mem.
             */
            NEW_EX(MmioCloseError);
            /**
             * @class MmioUnmapError
             * @param MmioUnmapError Error occured while unmapping memory.
             */
            NEW_EX(MmioUnmapError);
            /**
             * @class SpiArgError
             * @param SpiArgError Invalid arguments.
             */
            NEW_EX(SpiArgError);
            /**
             * @class SpiOpenError
             * @param SpiOpenError Error occured while opening SPI device.
             */
            NEW_EX(SpiOpenError);
            /**
             * @class SpiQueryError
             * @param SpiQueryError Error occured while querying SPI device settings.
             */
            NEW_EX(SpiQueryError);
            /**
             * @class SpiConfigureError
             * @param SpiConfigureError Error occured while configuring SPI device.
             */
            NEW_EX(SpiConfigureError);
            /**
             * @class SpiTransferError
             * @param SpiTransferError Error occured during SPI transfer.
             */
            NEW_EX(SpiTransferError);
            /**
             * @class SpiCloseError
             * @param SpiCloseError Error occured while closing SPI device.
             */
            NEW_EX(SpiCloseError);
    }
}

#endif //PERIPHERY_EXCEPTIONS_H
