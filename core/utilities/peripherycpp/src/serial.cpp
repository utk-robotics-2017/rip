#include "peripherycpp/serial.hpp"

namespace rip
{

    namespace peripherycpp
    {

        Serial::Serial()
        {}

        Serial::Serial(std::string device, unsigned int baudrate, unsigned int databits,
                  int parity, unsigned int stopbits,
                  bool xonxoff, bool rtscts)
        {
            open(device, baudrate, databits, parity, stopbits, xonxoff, rtscts);
        }

        Serial::~Serial()
        {
            close();
            devices_open[m_device] = false;
        }

        void Serial::open(std::string device, unsigned int baudrate, unsigned int databits,
                          int parity, unsigned int stopbits, bool xonxoff, bool rtscts)
        {
            misc::Logger::getInstance()->debug(fmt::format("Serial device {} opening", device));

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
            checkError(serial_open_advanced(&m_serial, device.c_str(), baudrate,
                          databits, cpar,stopbits, xonxoff, rtscts));

            misc::Logger::getInstance()->debug(fmt::format("Serial device {} successfully open", device));
            devices_open[device] = true;
            m_device = device;
            m_baudrate = baudrate;
            m_databits = databits;
            m_parity = parity;
            m_stopbits = stopbits;
            m_xonxoff = xonxoff;
            m_rtscts = rtscts;
        }

        std::vector<uint8_t> Serial::read(size_t len, int timeout_ms)
        {
            std::vector<uint8_t> data(static_cast<int>(len));
            checkError(serial_read(&m_serial, data.data(), len, timeout_ms));
            return data;
        }

        void Serial::read(char* buf, size_t size, int timeout_ms)
        {
            checkError(serial_read(&m_serial, (uint8_t*)buf , size, timeout_ms));
        }

        void Serial::read(uint8_t* buf, size_t size, int timeout_ms)
        {
            checkError(serial_read(&m_serial, buf, size, timeout_ms));
        }

        void Serial::write(std::vector<uint8_t> data)
        {
            checkError(serial_write(&m_serial, data.data(), data.size()));
        }

        void Serial::write(uint8_t* data, size_t size)
        {
            checkError(serial_write(&m_serial, data, size));
        }

        void Serial::write(char* data, size_t size)
        {
            checkError(serial_write(&m_serial, (uint8_t*)data, size));
        }

        void Serial::flush()
        {
            checkError(serial_flush(&m_serial));
        }

        unsigned int Serial::inputWaiting()
        {
            unsigned int count;
            checkError(serial_input_waiting(&m_serial, &count));
            return count;
        }

        unsigned int Serial::outputWaiting()
        {
            unsigned int count;
            checkError(serial_output_waiting(&m_serial, &count));
            return count;
        }

        bool Serial::poll(int timeout_ms)
        {
            bool pollval = serial_poll(&m_serial, timeout_ms);
            checkError(pollval);
            return pollval;
        }

        void Serial::close()
        {
            checkError(serial_close(&m_serial));
        }

        // Getters:
        uint32_t Serial::getBaudrate()
        {
            uint32_t baudrate;
            checkError(serial_get_baudrate(&m_serial, &baudrate));
            return baudrate;
        }

        unsigned int Serial::getDatabits()
        {
            unsigned int databits;
            checkError(serial_get_databits(&m_serial, &databits));
            return databits;
        }

        int Serial::getParity()
        {
            serial_parity_t parity;
            checkError(serial_get_parity(&m_serial, &parity));

            if (parity == PARITY_NONE)
            {
                return 0;
            }
            else if (parity == PARITY_ODD)
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
            unsigned int stopbits;
            checkError(serial_get_stopbits(&m_serial, &stopbits));
            return stopbits;
        }

        bool Serial::getxOnxOff()
        {
            bool xonxoff;
            checkError(serial_get_xonxoff(&m_serial, &xonxoff));
            return xonxoff;
        }

        bool Serial::getRtscts()
        {
            bool rtscts;
            checkError(serial_get_rtscts(&m_serial, &rtscts));
            return rtscts;
        }

        // Setters:
        void Serial::setBaudrate(uint32_t baudrate)
        {
            checkError(serial_set_baudrate(&m_serial, baudrate));
        }

        void Serial::setDatabits(unsigned int databits)
        {
            checkError(serial_set_baudrate(&m_serial, databits));
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

            checkError(serial_set_parity(&m_serial, cpar));
        }

        void Serial::setStopBits(unsigned int stopbits)
        {
                checkError(serial_set_stopbits(&m_serial, stopbits));
        }

        void Serial::setxOnxOff(bool enabled)
        {
            checkError(serial_set_xonxoff(&m_serial, enabled));
        }

        void Serial::setRtscts(bool enabled)
        {
            checkError(serial_set_rtscts(&m_serial, enabled));
        }

        void Serial::reset()
        {
            close();
            open(m_device, m_baudrate, m_databits, m_parity, m_stopbits, m_xonxoff, m_rtscts);
        }

        void Serial::checkError(int err_code)
        {
            switch(err_code)
            {
                case SERIAL_ERROR_ARG: throw SerialArgError(serial_errmsg(&m_serial)); break;
                case SERIAL_ERROR_OPEN: throw SerialOpenError(serial_errmsg(&m_serial)); break;
                case SERIAL_ERROR_QUERY: throw SerialQueryError(serial_errmsg(&m_serial)); break;
                case SERIAL_ERROR_IO: throw SerialIoError(serial_errmsg(&m_serial)); break;
                case SERIAL_ERROR_CONFIGURE: throw SerialConfigureError(serial_errmsg(&m_serial)); break;
                case SERIAL_ERROR_CLOSE: throw SerialCloseError(serial_errmsg(&m_serial)); break;
                default: /* no defined error */ break;
            }
        }

    }

}
