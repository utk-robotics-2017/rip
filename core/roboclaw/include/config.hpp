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
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <stdint.h>
#include "exceptions.hpp"

namespace rip
{
    namespace roboclaw
    {
        /**
         * @brief The Config class
         */
        class Config
        {
        public:
            enum class CommMode
            {
                kRC           = 0x0000,
                kAnalog       = 0x0001,
                kSimpleSerial = 0x0002,
                kPacketSerial = 0x0003
            };

            /**
             * @brief Sets the comm mode for the config
             * @param comm_mode The comm mode to set
             */
            void setCommMode(CommMode comm_mode);

            /**
             * @brief Returns the comm mode for the config
             * @return The comm mode
             */
            CommMode getCommMode() const;

            enum class BatteryMode
            {
                kOff   = 0x0000,
                kAuto  = 0x0004,
                k2Cell = 0x0008,
                k3Cell = 0x000c,
                k4Cell = 0x0010,
                k5Cell = 0x0014,
                k6Cell = 0x0018,
                k7Cell = 0x001c
            };
            /**
             * @brief Sets the battery mode
             * @param The battery mode to set
             */
            void setBatteryMode(BatteryMode battery_mode);

            /**
             * @brief Returns the battery mode for this config
             * @returns The battery mode
             */
            BatteryMode getBatteryMode() const;

            enum class BaudRate
            {
                k2400   = 0x0000,
                k9600   = 0x0020,
                k19200  = 0x0040,
                k38400  = 0x0060,
                k57600  = 0x0080,
                k115200 = 0x00a0,
                k230400 = 0x00c0,
                k460800 = 0x00e0
            };

            /**
             * @brief Sets the baud rate
             * @param baud_rate The baud rate to set
             */
            void setBaudRate(BaudRate baud_rate);

            /**
             * @brief Returns the baud rate
             * @return The baud rate for this config
             */
            BaudRate getBaudRate() const;

            enum class PacketAddress
            {
                k80 = 0x0000,
                k81 = 0x0100,
                k82 = 0x0200,
                k83 = 0x0300,
                k84 = 0x0400,
                k85 = 0x0500,
                k86 = 0x0600,
                k87 = 0x0700,
            };

            /**
             * @brief Sets the packet address
             * @param packet_address
             */
            void setPacketAddress(PacketAddress packet_address);

            /**
             * @brief Returns the packet address
             * @returns The packet address
             */
            PacketAddress getPacketAddress() const;

            /**
             * @brief Set whether the encoders are swapped
             * @param swap
             */
            void setSwapEncoders(bool swap);

            /**
             * @brief Set whether the buttons are swapped
             * @param swap
             */
            void setSwapButtons(bool swap);

            /**
             * @brief Set the config
             * @param config
             */
            void set(uint16_t config);
            /**
             * @brief Get the config
             * @param n/a
             */
            uint16_t get() const;
        private:
            uint16_t m_config = 0x80A7;
        };
    }
}

#endif // CONFIG_HPP
