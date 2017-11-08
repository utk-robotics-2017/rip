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
#ifndef ROBOCLAW_EXCEPTIONS_HPP
#define ROBOCLAW_EXCEPTIONS_HPP
#include <exception_base.hpp>

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            /**
             * @class CommandFailure
             * @brief An exception for when the command fails
             */
            NEW_EX(CommandFailure);

            /**
             * @class ReadFailure
             * @brief An exception for when the response for a read command fails
             */
            NEW_EX(ReadFailure);

            /**
             * @class OutOfRange
             * @brief An exception for when something falls out of range
             */
            NEW_EX(OutOfRange);

            /**
             * @class UnknownDType
             * @brief Unknown combination of motor dynamic parameters
             */
            NEW_EX(UnknownDType);
            /**
             * Json passed to Roboclaw constructor is null or has bad data.
             */
            NEW_EX(BadJson);
            /*
            Exception due to a communication failure, device may be offline, o
            or not connected via serial connections, or powered off.
            */

        }
    }
}
#endif //ROBOCLAW_EXCEPTIONS_HPP
