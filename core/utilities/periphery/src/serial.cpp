#include "serial.hpp"

namespace rip
{
    namespace periphery
    {
        void Serial::open(std::string device, unsigned int baudrate)
        {
            if(serial_open(m_serial, device.c_str(), baudrate) < 0)
            {
                throw SerialOpenFail(serial_errmsg(m_serial));
            }
        }

        void Serial::open(std::string device, unsigned int baudrate, unsigned int databits,
        serial_parity_t parity, unsigned int stopbits,
        bool xonxoff, bool rtscts)
        {
            if(serial_open_advanced(m_serial, device.c_str(), baudrate, databits, parity,
                stopbits, xonxoff, rtscts) < 0)
            {
                throw SerialOpenFail(serial_errmsg(m_serial));
            }
        }

        std::vector<uint8_t> Serial::read(size_t len, int timeout_ms)
        {
            std::vector<uint8_t> data(static_cast<int>(len));
            if(serial_read(m_serial, &data[0], len, timeout_ms) < 0)
            {
                throw SerialReadFailure(serial_errmsg(m_serial));
            }
        }

        void Serial::write(std::vector<uint8_t> data)
        {
            if(serial_write(m_serial, &data[0], data.size()) < 0)
            {
                throw SerialWriteFailure(serial_errmsg(serial));
            }
        }
    }
}
