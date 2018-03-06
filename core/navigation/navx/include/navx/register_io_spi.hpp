/*
 * RegisterIOSPI.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott & UTK IEEE Robotics
 */

#ifndef SRC_REGISTERIOSPI_H_
#define SRC_REGISTERIOSPI_H_

#include "register_io.hpp"
#include <peripherycpp/spi.hpp>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {

            static const int MAX_SPI_MSG_LENGTH = 256;

            class RegisterIO_SPI: public IRegisterIO
            {
            public:
                RegisterIO_SPI(peripherycpp::Spi *port, uint32_t bitrate);
                virtual ~RegisterIO_SPI() {}
                bool init();
                bool write(uint8_t address, uint8_t value );
                bool read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len);
                bool shutdown();
                void enableLogging(bool enable);
            private:
                peripherycpp::Spi port;
                uint32_t bitrate;
                uint8_t rx_buffer[MAX_SPI_MSG_LENGTH];
                bool trace;
            };
        }
    }
}

#endif /* SRC_REGISTERIOSPI_H_ */
