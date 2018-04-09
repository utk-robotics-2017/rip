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
#ifndef CMD_MESSENGER_EXCEPTIONS_HPP
#define CMD_MESSENGER_EXCEPTIONS_HPP
#include <misc/exception_base.hpp>

namespace rip
{
    namespace cmdmessenger
    {
        NEW_EX(EmptyDevice)
        NEW_EX(EmptyCommand)
        NEW_EX(EmptyDeviceResponse)

        NEW_EX(IncorrectAcknowledgementCommand)
        NEW_EX(IncorrectArgumentListSize)
        NEW_EX(IncorrectCommandSeparator)
        NEW_EX(IncorrectFieldSeparator)
        NEW_EX(IncorrectResponseCommand)
        NEW_EX(OutOfBounds)
        NEW_EX(NoLastDeviceException)
        NEW_EX(UnconvertibleArgument)
        NEW_EX(UnknownArgument)
        NEW_EX(DeviceSentErrorResponse)

        NEW_EX(SerialLibrarySucks)
    }
}
#endif // CMD_MESSENGER_EXCEPTIONS_HPP
