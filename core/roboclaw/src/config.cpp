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
#include "config.hpp"
#include <iostream>

namespace rip
{
    namespace roboclaw
    {
        void Config::setCommMode(CommMode comm_mode)
        {
            // Clear comm mode using bit mask
            uint16_t clear = 0xFFFC;
            m_config &= clear;

            m_config |= static_cast<uint16_t>(comm_mode);
        }

        Config::CommMode Config::getCommMode() const
        {
            return static_cast<CommMode>(m_config & 0x03);
        }

        void Config::setBatteryMode(BatteryMode battery_mode)
        {
            // Clear battery mode using bit mask
            uint16_t clear = 0xFFE3;
            m_config &= clear;

            m_config |= static_cast<uint16_t>(battery_mode);
        }

        Config::BatteryMode Config::getBatteryMode() const
        {

            return static_cast<BatteryMode>(m_config & 0x1C);
        }

        void Config::setBaudRate(BaudRate baud_rate)
        {
            if (static_cast<uint32_t>(this->getCommMode()) <= 1)
            {
                throw InvalidCommMode("Baudrate cannot be set when commMode is RC/Analog");
            }
            // Clear baud rate using bit mask
            uint16_t clear = 0xFF1F;
            m_config &= clear;

            m_config |= static_cast<uint16_t>(baud_rate);
        }

        Config::BaudRate Config::getBaudRate() const
        {
            switch (static_cast<uint32_t>(this->getCommMode()))
            {
                case 0:
                case 1:
                    throw InvalidCommMode("commMode is RC/Analog");
                case 2:
                    throw InvalidCommMode("Simple serial is write only");
            }
            return static_cast<BaudRate>(m_config & 0xE0);
        }

        void Config::setPacketAddress(PacketAddress packet_address)
        {
            if (static_cast<uint32_t>(this->getCommMode()) <= 1)
            {
                throw InvalidCommMode("PacketAddress cannot be set when commMode is RC/Analog");
            }
            // Clear packet address using bit mask
            uint16_t clear = 0xF0FF;
            m_config &= clear;

            m_config |= static_cast<uint16_t>(packet_address);
        }

        Config::PacketAddress Config::getPacketAddress() const
        {
            switch (static_cast<uint32_t>(this->getCommMode()))
            {
                case 0:
                case 1:
                    throw InvalidCommMode("commMode is RC/Analog");
                case 2:
                    throw InvalidCommMode("Simple serial is write only");
            }
            return static_cast<PacketAddress>(m_config & 0xF00);
        }

        void Config::setSwapEncoders(bool swap)
        {
            // Clear swap encoders using bit mask
            uint16_t clear = 0xDFFF;
            m_config &= clear;

            if (swap)
            {
                m_config |= 0x2000;
            }
        }

        void Config::setSwapButtons(bool swap)
        {
            // Clear swap buttons using bit mask
            uint16_t clear = 0xBFFF;
            m_config &= clear;

            if (swap)
            {
                m_config |= 0x4000;
            }
        }

        void Config::set(uint16_t config)
        {
            m_config = config;
        }

        uint16_t Config::get() const
        {
            return m_config;
        }
    }
}
