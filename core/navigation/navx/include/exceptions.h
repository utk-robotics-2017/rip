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
#ifndef NAVX_EXCEPTIONS_H
#define NAVX_EXCEPTIONS_H
#include <exception_base.hpp>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            /**
             * @class SerialOpen
             * @brief NavxSerialIo failed to open
             */
            NEW_EX(SerialOpen);
            /**
             * @class SerialReset
             * @brief NavxSerialIo failed to open
             */
            NEW_EX(SerialReset);
             /**
              * @class SerialNotOpen
              * @brief Attempted to run things on a serial that is not open
              */
            NEW_EX(SerialNotOpen);
             /**
              * @class SerialFail
              * @brief Serial communications have failed during runtime.
              */
            NEW_EX(SerialFail);
            /**
              * @class SerialEncoding
              * @brief Serial communications have failed while encoding data.
              */
            NEW_EX(SerialEncoding);
            /**
              * @class SerialSend
              * @brief erial communications have failed during transmission due to
              * an issue with integration control.
              */
            NEW_EX(SerialIntegrationControl);

        }
    }
}

#endif //NAVX_EXCEPTIONS_H
