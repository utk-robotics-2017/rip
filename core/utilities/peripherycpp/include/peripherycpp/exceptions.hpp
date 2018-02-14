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
             * @brief Brief description of exception
             */
            NEW_EX(ExampleName);
            /**
             * @class SerialOpenFail
             * @param SerialOpenFail serial fails to open
             */
            NEW_EX(SerialOpenFail);
            /**
             * @class SerialReadFailure
             * @param SerialReadFailure failure to read bytes
             */
            NEW_EX(SerialReadFailure);
            /**
             * @class SerialWriteFailure
             * @param SerialWriteFailure serial writing fails
             */
            NEW_EX(SerialWriteFailure);
            /**
             * @class SerialFlushFailure
             * @param SerialFlushFailure serial fails to flush
             */
            NEW_EX(SerialFlushFailure);
            /**
             * @class SerialCloseFailure
             * @param SerialCloseFailure serial fails to close
             */
            NEW_EX(SerialCloseFailure);
            /**
             * @class SerialGetFailure
             * @param SerialGetFailure failed to get data from serial struct
             */ 
            NEW_EX(SerialGetFailure);
            /**
             * @class SerialSetFailure
             * @param SerialSetFailure failed to set data in serial struct
             */
            NEW_EX(SerialSetFailure);
    }
}

#endif //PERIPHERY_EXCEPTIONS_H
