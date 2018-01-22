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

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <string>
#include <serial/serial.h>
#include <units/units.hpp>

namespace rip
{
    namespace cmdmessenger
    {
        /**
             * @class Device
             * @brief
             */
        class Device : public serial::Serial
        {
        public:
            /**
                 * @brief Constructor
                 *
                 * @param port The port for the serial connection
                 * @param baud_rate The speed for sending and receiving messages
                 * @param timeout The timeout to allow for communication
                 */
            Device(std::string port, unsigned long baud_rate = 115200, units::Time timeout = 1.0);

            /**
                 * @brief Sets the timeout to allow for communication
                 * @param timeout The timeout to set
                 */
            void setTimeout(units::Time timeout);
        };
    }
}
#endif // DEVICE_HPP
