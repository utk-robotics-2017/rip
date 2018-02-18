#include "peripherycpp/spi.hpp"
#include <string>

namespace rip
{

    namespace periphery
    {

        void Spi::open(const std::string path, unsigned int mode, uint32_t max_speed)
        {
            const char *cpath = path.c_str();
            int err_num = spi_open(&spi, cpath, mode, max_speed);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -2: throw SpiOpenError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::openAdvanced(const std::string path, unsigned int mode, uint32_t max_speed,
                                int bit_order, uint8_t bits_per_word, uint8_t extra_flags)
        {
            const char *cpath = path.c_str();
            spi_bit_order_t bo;
            if (bit_order == 0)
            {
                bo = MSB_FIRST;
            }
            else
            {
                bo = LSB_FIRST;
            }
            int err_num = spi_open_advanced(&spi, cpath, mode, max_speed, bo, bits_per_word, extra_flags);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -2: throw SpiOpenError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
        {
            int err_num = spi_transfer(&spi, txbuf, rxbuf, len);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -5: throw SpiTransferError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::close()
        {
            int err_num = spi_close(&spi);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -6: throw SpiCloseError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        unsigned int Spi::getMode()
        {
            unsigned int *mode;
            int err_num = spi_get_mode(&spi, mode);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                default: break;
            }
            return *mode;
        }

        uint32_t Spi::getMaxSpeed()
        {
            uint32_t *max_speed;
            int err_num = spi_get_max_speed(&spi, max_speed);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                default: break;
            }
            return *max_speed;
        }

        int Spi::getBitOrder()
        {
            spi_bit_order_t *bit_order;
            int err_num = spi_get_bit_order(&spi, bit_order);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                default: break;
            }
            int bo;
            if (*bit_order == MSB_FIRST)
            {
                bo = 0;
            }
            else
            {
                bo = 1;
            }
            return bo;
        }

        uint8_t Spi::getBitsPerWord()
        {
            uint8_t *bpw;
            int err_num = spi_get_bits_per_word(&spi, bpw);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                default: break;
            }
            return *bpw;
        }

        uint8_t Spi::getExtraFlags()
        {
            uint8_t *eflags;
            int err_num = spi_get_extra_flags(&spi, eflags);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                default: break;
            }
            return *eflags;
        }

        void Spi::setMode(unsigned int mode)
        {
            int err_num = spi_set_mode(&spi, mode);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::setMaxSpeed(uint32_t max_speed)
        {
            int err_num = spi_set_max_speed(&spi, max_speed);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
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
            int err_num = spi_set_bit_order(&spi, bo);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::setBitsPerWord(uint8_t bits_per_word)
        {
            int err_num = spi_set_bits_per_word(&spi, bits_per_word);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        void Spi::setExtraFlags(uint8_t extra_flags)
        {
            int err_num = spi_set_extra_flags(&spi, extra_flags);
            switch(err_num)
            {
                case -1: throw SpiArgError(spi_errmsg(&spi));
                case -3: throw SpiQueryError(spi_errmsg(&spi));
                case -4: throw SpiConfigureError(spi_errmsg(&spi));
                default: break;
            }
            return;
        }

        int Spi::fd()
        {
            return spi_fd(&spi);
        }

        std::string Spi::toString(size_t len)
        {
            char *cstr;
            spi_tostring(&spi, cstr, len);
            std::string str = cstr;
            return str;
        }

    }

}
