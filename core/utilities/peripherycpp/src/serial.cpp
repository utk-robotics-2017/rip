#include "peripherycpp/serial.hpp"

namespace rip
{

    namespace peripherycpp
    {

        // Primary Functions:
        void Serial::open(std::string device, unsigned int baudrate)
        {
            if(serial_open(&m_serial, device.c_str(), baudrate) < 0)
            {
                throw SerialOpenFail(serial_errmsg(&m_serial));
            }
        }

        void Serial::open(std::string device, unsigned int baudrate, unsigned int databits,
                          int parity, unsigned int stopbits, bool xonxoff, bool rtscts)
        {
            serial_parity_t cpar;
            if (parity == 0)
            {
                cpar = PARITY_NONE;
            }
            else if (parity == 1)
            {
                cpar = PARITY_ODD;
            }
            else
            {
                cpar = PARITY_EVEN;
            }
            if(serial_open_advanced(&m_serial, device.c_str(), baudrate, databits, cpar,
                stopbits, xonxoff, rtscts) < 0)
            {
                throw SerialOpenFail(serial_errmsg(&m_serial));
            }
        }

        std::vector<uint8_t> Serial::read(size_t len, int timeout_ms)
        {
            std::vector<uint8_t> data(static_cast<int>(len));
            if(serial_read(&m_serial, &data[0], len, timeout_ms) < 0)
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::write(std::vector<uint8_t> data)
        {
            if(serial_write(&m_serial, &data[0], data.size()) < 0)
            {
                throw SerialWriteFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::flush() 
        {
            if(serial_flush(&m_serial) < 0) 
            {
                throw SerialFlushFailure(serial_errmsg(&m_serial));
            }
        }

        unsigned int Serial::inputWaiting()
        {
            unsigned int *count;
            if(serial_input_waiting(&m_serial, count) < 0)
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
            return *count;
        }

        unsigned int Serial::outputWaiting() 
        {
            unsigned int *count;
            if(serial_output_waiting(&m_serial, count) < 0)
            {
                throw SerialWriteFailure(serial_errmsg(&m_serial));
            }
            return *count;
        }

        bool Serial::poll(int timeout_ms)
        {
            bool pollval = serial_poll(&m_serial, timeout_ms);
            if(pollval >= 0)
            {
                return pollval;
            }
            else
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::close()
        {
            if(serial_close(&m_serial) < 0)
            {
                throw SerialCloseFailure(serial_errmsg(&m_serial));
            }
        }

        // Getters:
        uint32_t Serial::getBaudrate()
        {
            uint32_t *baudrate;
            if(serial_get_baudrate(&m_serial, baudrate) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *baudrate;
        }

        unsigned int Serial::getDatabits()
        {
            unsigned int *databits;
            if(serial_get_databits(&m_serial, databits) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *databits;
        }

        int Serial::getParity()
        {
            serial_parity_t *parity;
            if(serial_get_parity(&m_serial, parity) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            if (*parity == PARITY_NONE)
            {
                return 0;
            }
            else if (*parity == PARITY_ODD)
            {
                return 1;
            }
            else
            {
                return 2;
            }
        }

        unsigned int Serial::getStopbits()
        {
            unsigned int *stopbits;
            if (serial_get_stopbits(&m_serial, stopbits) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *stopbits;
        }

        bool Serial::getxOnxOff()
        {
            bool *xonxoff;
            if (serial_get_xonxoff(&m_serial, xonxoff) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *xonxoff;
        }

        bool Serial::getRtscts()
        {
            bool *rtscts;
            if (serial_get_rtscts(&m_serial, rtscts) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *rtscts;
        }
        
        // Setters:
        void Serial::setBaudrate(uint32_t baudrate)
        {
            if (serial_set_baudrate(&m_serial, baudrate) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::setDatabits(unsigned int databits)
        {
            if (serial_set_baudrate(&m_serial, databits) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::setParity(int parity)
        {
            serial_parity_t cpar;
            if (parity == 0)
            {
                cpar = PARITY_NONE;
            }
            else if (parity == 1)
            {
                cpar = PARITY_ODD;
            }
            else
            {
                cpar = PARITY_EVEN;
            }
            if (serial_set_parity(&m_serial, cpar) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::setxOnxOff(bool enabled)
        {
            if (serial_set_xonxoff(&m_serial, enabled) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::setRtscts(bool enabled)
        {
            if (serial_set_rtscts(&m_serial, enabled) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }

    }

}
