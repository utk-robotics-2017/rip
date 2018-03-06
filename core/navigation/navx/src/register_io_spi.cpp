/*
 * RegisterIOSPI.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#include <navx/register_io_spi.hpp>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            /*
            RegisterIO_SPI::RegisterIO_SPI(SPI *port, uint32_t bitrate)
            {
                this->port = port;
                this->bitrate = bitrate;
                this->trace = false;
            }

            bool RegisterIO_SPI::init()
            {
                port->setClockRate(bitrate);
                port->setMSBFirst();
                port->setSampleDataOnFalling();
                port->setClockActiveLow();
                port->setChipSelectActiveLow();
                if(trace)printf("navX-MXP:  Initialized SPI communication at bitrate %d\n", bitrate);
                return true;
            }

            bool RegisterIO_SPI::write(uint8_t address, uint8_t value)
            {
                uint8_t cmd[3];
                cmd[0] = address | 0x80;
                cmd[1] = value;
                cmd[2] = IMURegisters::getCRC(cmd, 2);
                if(port->write(cmd, sizeof(cmd))!= sizeof(cmd))
                {
                    return false; // WRITE ERROR
                }
                return true;
            }
            */
           /*
            bool RegisterIO_SPI::read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len)
            {
                uint8_t cmd[3];
                cmd[0] = first_address;
                cmd[1] = buffer_len;
                cmd[2] = IMURegisters::getCRC(cmd, 2);
                if(port->write(cmd, sizeof(cmd))!= sizeof(cmd))
                {
                    return false; // WRITE ERROR
                }
                // delay 200 us TODO:  What is min. granularity of delay()?
                wait(0.001);
                if(port->read(true, rx_buffer, buffer_len+1)!= buffer_len+1)
                {
                    return false; // READ ERROR
                }
                uint8_t crc = IMURegisters::getCRC(rx_buffer, buffer_len);
                if(crc != rx_buffer[buffer_len])
                {
                    return false; // CRC ERROR
                }
                else
                {
                    memcpy(buffer, rx_buffer, buffer_len);
                }
                return true;
            }

            bool RegisterIO_SPI::shutdown()
            {
                return true;
            }

            void RegisterIO_SPI::enableLogging(bool enable)
            {
                trace = enable;
            }
            */
        }
    }
}
