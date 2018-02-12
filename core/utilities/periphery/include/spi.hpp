#ifndef _PERIPHERY_SPI_HPP
#define _PERIPHERY_SPI_HPP

extern "C"
{
<<<<<<< HEAD
    #include "spi.h"
=======
    #include "serial.h"
>>>>>>> 2f57d3d4c4282211acf60e6e2f1f19b1184c621f
}

namespace rip
{
<<<<<<< HEAD

    namespace periphery
    {

        class Spi
        {
            public:
                void open(const std::string path, unsigned int mode, uint32_t max_speed);
                void openAdvanced(const std::string path, unsigned int mode, uint32_t max_speed,
                                   int bit_order, uint8_t bits_per_word, uint8_t extra_flags);
                void transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len);
                void close();
                unsigned int getMode();
                uint32_t getMaxSpeed();
                int getBitOrder();
                uint8_t getBitsPerWord();
                uint8_t getExtraFlags();
                void setMode(unsigned int mode);
                void setMaxSpeed(uint32_t max_speed);
                void setBitOrder(int bit_order);
                void setBitsPerWord(uint8_t bits_per_word);
                void setExtraFlags(uint8_t extra_flags);
                int fd();
                std::string toString(size_t len);
            private:
                spi_t spi;
        };

    }

}
=======
    namespace periphery
    {
        class Spi
        {
            
        }
    }
}

#endif
>>>>>>> 2f57d3d4c4282211acf60e6e2f1f19b1184c621f
