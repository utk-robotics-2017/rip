#ifndef _PERIPHERY_SERIAL_HPP
#define _PERIPHERY_SERIAL_HPP
#include <string>
#include <vector>
#include <stdint>
#include "exceptions.h"

extern "C"
{
    #include "serial.h"
}

namespace rip
{

    namespace periphery
    {
        class Serial
        {
        public:
            /**
             * open
             * @param device   tty device at the specified path (e.g. "/dev/ttyUSB0")
             * @param baudrate device baudrate (ie, 115200).
             * @brief
             */
            void open(std::string device, unsigned int baudrate);
            /**
             * [open description]
             * @param device   tty device at the specified path (e.g. "/dev/ttyUSB0")
             * @param baudrate baudrate device baudrate (ie, 115200).
             * @param databits databits can be 5, 6, 7, or 8
             * @param parity   parity can be PARITY_NONE, PARITY_ODD, or PARITY_EVEN
             * @param stopbits can be 1 or 2
             * @param xonxoff  [description]
             * @param rtscts   [description]
             * @brief Open the tty device at the specified path (e.g. "/dev/ttyUSB0"),
             * with the specified baudrate, data bits, parity, stop bits, software flow control (xonxoff),
             * and hardware flow control (rtscts) settings.
             */
            void open(std::string device, unsigned int baudrate, unsigned int databits,
            serial_parity_t parity, unsigned int stopbits,
            bool xonxoff, bool rtscts);
        private:
            serial_t serial;
        }
    }
}

#endif
