#include "peripherycpp/spi.hpp"
#include <string>

namespace rip
{
    namespace peripherycpp
    {
        void Spi::open(const std::string path, unsigned int mode, uint32_t max_speed)
        {
            checkError(spi_open(&m_spi, path.c_str(), mode, max_speed));
        }

        void Spi::openAdvanced(const std::string path, unsigned int mode, uint32_t max_speed,
                                int bit_order, uint8_t bits_per_word, uint8_t extra_flags)
        {
            spi_bit_order_t bo;
            if (bit_order == 0)
            {
                bo = MSB_FIRST;
            }
            else
            {
                bo = LSB_FIRST;
            }
            checkError(spi_open_advanced(&m_spi, path.c_str(), mode, max_speed, bo,
                                         bits_per_word, extra_flags));
        }

        void Spi::transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
        {
            checkError(spi_transfer(&m_spi, txbuf, rxbuf, len));
        }

        void Spi::close()
        {
            checkError(spi_close(&m_spi));
        }

        unsigned int Spi::getMode()
        {
            unsigned int mode;
            checkError(spi_get_mode(&m_spi, &mode));
            return mode;
        }

        uint32_t Spi::getMaxSpeed()
        {
            uint32_t max_speed;
            checkError(spi_get_max_speed(&m_spi, &max_speed));
            return max_speed;
        }

        int Spi::getBitOrder()
        {
            spi_bit_order_t bit_order;
            checkError(spi_get_bit_order(&m_spi, &bit_order));
            return (bit_order == MSB_FIRST);
        }

        uint8_t Spi::getBitsPerWord()
        {
            uint8_t bpw;
            checkError(spi_get_bits_per_word(&m_spi, &bpw));
            return bpw;
        }

        uint8_t Spi::getExtraFlags()
        {
            uint8_t eflags;
            checkError(spi_get_extra_flags(&m_spi, &eflags));
            return eflags;
        }

        void Spi::setMode(unsigned int mode)
        {
            checkError(spi_set_mode(&m_spi, mode));
        }

        void Spi::setMaxSpeed(uint32_t max_speed)
        {
            checkError(spi_set_max_speed(&m_spi, max_speed));
        }

        void Spi::setBitOrder(int bit_order)
        {
            spi_bit_order_t bo;
            if (bit_order == 0)
            {
                bo = MSB_FIRST;
            }
            else
            {
                bo = LSB_FIRST;
            }
            checkError(spi_set_bit_order(&m_spi, bo));
        }

        void Spi::setBitsPerWord(uint8_t bits_per_word)
        {
            checkError(spi_set_bits_per_word(&m_spi, bits_per_word));
        }

        void Spi::setExtraFlags(uint8_t extra_flags)
        {
            checkError(spi_set_extra_flags(&m_spi, extra_flags));
        }

        int Spi::fd()
        {
            return spi_fd(&m_spi);
        }

        std::string Spi::toString(size_t len)
        {
            char *cstr = new char[len];
            spi_tostring(&m_spi, cstr, len);
            std::string ret(cstr);
            delete [] cstr;
            return ret;
        }

        void Spi::checkError(int err_code)
        {
            switch(err_code)
            {
                case SPI_ERROR_ARG: throw SpiArgError(spi_errmsg(&m_spi));
                case SPI_ERROR_OPEN: throw SpiOpenError(spi_errmsg(&m_spi));
                case SPI_ERROR_QUERY: throw SpiQueryError(spi_errmsg(&m_spi));
                case SPI_ERROR_CONFIGURE: throw SpiConfigureError(spi_errmsg(&m_spi));
                case SPI_ERROR_TRANSFER: throw SpiTransferError(spi_errmsg(&m_spi));
                case SPI_ERROR_CLOSE: throw SpiCloseError(spi_errmsg(&m_spi));
                default: /* no defined error */ break;
            }
        }
    }
}
