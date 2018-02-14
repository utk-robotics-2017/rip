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

    namespace periphery
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
             * @param I2cOpenError Error opening I2C device.
             */
            NEW_EX(I2cOpenError);
            /**
             * @class I2cQuerySupportError
             * @param I2cQuerySupportError Error querying I2C support on I2C device.
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
             * @param I2cCloseError Error closing the I2C device.
             */
            NEW_EX(I2cCloseError);
    }
}

#endif //PERIPHERY_EXCEPTIONS_H
