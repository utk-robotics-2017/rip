#include <periphery_cpp/serial.hpp>

namespace rip
{
    namespace peripherycpp
    {
        // Primary Functions:
        void Serial::Open(std::string device, unsigned int baudrate)
        {
            if(serial_open(&m_serial, device.c_str(), baudrate) < 0)
            {
                throw SerialOpenFail(serial_errmsg(&m_serial));
            }
        }
        void Serial::Open(std::string device, unsigned int baudrate, unsigned int databits,
        serial_parity_t parity, unsigned int stopbits,
        bool xonxoff, bool rtscts)
        {
            if(serial_open_advanced(&m_serial, device.c_str(), baudrate, databits, parity,
                stopbits, xonxoff, rtscts) < 0)
            {
                throw SerialOpenFail(serial_errmsg(&m_serial));
            }
        }
        std::vector<uint8_t> Serial::Read(size_t len, int timeout_ms)
        {
            std::vector<uint8_t> data(static_cast<int>(len));
            if(serial_read(&m_serial, &data[0], len, timeout_ms) < 0)
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
        }

        void Serial::Write(std::vector<uint8_t> data)
        {
            if(serial_write(&m_serial, &data[0], data.size()) < 0)
            {
                throw SerialWriteFailure(serial_errmsg(&m_serial));
            }
        }
		void Serial::Flush() 
		{
			if(serial_flush(&m_serial) < 0) 
			{
				throw SerialFlushFailure(serial_error(&m_serial));
			}
		}
        void Serial::Input_Waiting(unsigned int *count)
        {
            if(serial_input_waiting(&m_serial, count)
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Output_Waiting(unsigned int *count) 
        {
            if(serial_output_waiting(&m_serial, count) < 0)
            {
                throw SerialWriteFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Poll(int timeout_ms)
        {
            if(serial_poll(&m_serial, timeout_ms) < 0)
            {
                throw SerialReadFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Close()
        {
            if(serial_close(&m_serial)) < 0)
            {
                throw SerialCloseFailure(serial_errmsg(&m_serial));
            }
        }

        // Getters:
        uint32_t Serial::Get_Baudrate()
        {
            uint32_t *baudrate;
            if(serial_get_baudrate(&m_serial, baudrate)) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *baudrate;
        }
        unsigned int Serial::Get_Databits()
        {
            unsigned int *databits
            if(serial_get_databits(&m_serial, databits)) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *databits;
        }
        serial_parity_t Serial::Get_Parity()
        {
            serial_parity_t *parity
            if(serial_get_parity(&m_serial, *parity)) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *parity;
        }
        unsigned int Serial::Get_Stopbits()
        {
            unsigned int *stopbits
            if (serial_get_stopbits(&m_serial, stopbits) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *stopbits;
        }
        bool Serial::Get_xOnxOff()
        {
            bool *xonxoff;
            if (serial_get_xonxoff(&m_serial, xonxoff) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *xonxoff;
        }
        bool Serial::Get_Rtscts()
        {
            bool *rtscts;
            if (serial_get_rtscts(&m_serial, rtscts) < 0)
            {
                throw SerialGetFailure(serial_errmsg(&m_serial));
            }
            return *rtscts;
        }
        
        // Setters:
        void Serial::Set_Baudrate(uint32_t baudrate)
        {
            if (serial_set_baudrate(&m_serial, baudrate) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Set_Databits(unsigned int databits)
        {
            if (serial_set_baudrate(&m_serial, databits) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Set_Parity(emun serial_parity parity)
        {
            if (serial_set_parity(&m_serial, parity) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Set_xOnxOff(bool enabled)
        {
            if (serial_set_xonxoff(&m_serial, enabled) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }
        void Serial::Set_Rtscts(bool enabled)
        {
            if (serial_set_rtscts(&m_serial, enabled) < 0)
            {
                throw SerialSetFailure(serial_errmsg(&m_serial));
            }
        }
    }
}
