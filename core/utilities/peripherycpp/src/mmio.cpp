#include "peripherycpp/mmio.hpp"
#include <cstdint>
#include <vector>

namespace rip
{

    namespace peripherycpp
    {

        void Mmio::open(uintptr_t base, size_t size)
        {
            int err_num = mmio_open(&mmio, base, size);
            switch(err_num)
            {
                case -1: throw MmioArgError(mmio_errmsg(&mmio));
                case -2: throw MmioOpenError(mmio_errmsg(&mmio));
                case -3: throw MmioMapError(mmio_errmsg(&mmio));
                default: break;
            }
            return;
        }

        uint32_t Mmio::read32(uintptr_t offset)
        {
            uint32_t value;
            int err_num = mmio_read32(&mmio, offset, &value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
            return value;
        }

        uint16_t Mmio::read16(uintptr_t offset)
        {
            uint16_t value;
            int err_num = mmio_read16(&mmio, offset, &value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
            return value;
        }

        uint8_t Mmio::read8(uintptr_t offset)
        {
            uint8_t value;
            int err_num = mmio_read8(&mmio, offset, &value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
            return value;
        }

        std::vector<uint8_t> Mmio::read(uintptr_t offset, size_t len)
        {
            std::vector<uint8_t> buf(static_cast<int>(len));
            int err_num = mmio_read(&mmio, offset, &buf[0], len);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
            return buf;
        }

        void Mmio::write32(uintptr_t offset, uint32_t value)
        {
            int err_num = mmio_write32(&mmio, offset, value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
        }

        void Mmio::write16(uintptr_t offset, uint16_t value)
        {
            int err_num = mmio_write16(&mmio, offset, value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
        }

        void Mmio::write8(uintptr_t offset, uint8_t value)
        {
            int err_num = mmio_write8(&mmio, offset, value);
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
        }

        void Mmio::write(uintptr_t offset, std::vector<uint8_t> &buf)
        {
            int err_num = mmio_write(&mmio, offset, &buf[0], buf.size());
            if (err_num == -1)
            {
                throw MmioArgError(mmio_errmsg(&mmio));
            }
        }

        void Mmio::close()
        {
            int err_num = mmio_close(&mmio);
            switch(err_num)
            {
                case -1: throw MmioArgError(mmio_errmsg(&mmio)); break;
                case -4: throw MmioCloseError(mmio_errmsg(&mmio)); break;
                case -5: throw MmioUnmapError(mmio_errmsg(&mmio)); break;
                default: break;
            }
        }

        uintptr_t Mmio::base()
        {
            return mmio_base(&mmio);
        }

        size_t Mmio::size()
        {
            return mmio_size(&mmio);
        }

        std::string Mmio::toString(size_t len)
        {
            char *cstr = new char[len];
            mmio_tostring(&mmio, cstr, len);
            return std::string(cstr);
        }

    }

}
