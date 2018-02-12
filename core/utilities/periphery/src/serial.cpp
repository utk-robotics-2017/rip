#include "serial.hpp"

namespace rip
{
    namespace periphery
    {
        void Serial::open(std::string device, unsigned int baudrate)
        {
            if(serial_open(serial, device.c_str(), baudrate) < 0)
            {
                throw SerialOpenFail(serial_errmsg(serial));
            }
        }

        void Serial::open(std::string device, unsigned int baudrate, unsigned int databits,
        serial_parity_t parity, unsigned int stopbits,
        bool xonxoff, bool rtscts)
        {
            if(serial_open_advanced(serial, device.c_str(), baudrate, databits, parity,
                stopbits, xonxoff, rtscts) < 0)
            {
                throw SerialOpenFail(serial_errmsg(serial));
            }
        }
    }
}
