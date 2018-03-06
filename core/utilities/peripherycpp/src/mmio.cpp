#include "peripherycpp/mmio.hpp"
#include <cstdint>
#include <vector>

namespace rip
{

    namespace peripherycpp
    {

        void Mmio::open(uintptr_t base, size_t size)
        {
            checkError(mmio_open(&m_mmio, base, size));
        }

        uint32_t Mmio::read32(uintptr_t offset)
        {
            uint32_t value;
            checkError(mmio_read32(&m_mmio, offset, &value));
            return value;
        }

        uint16_t Mmio::read16(uintptr_t offset)
        {
            uint16_t value;
            checkError(mmio_read16(&m_mmio, offset, &value));
            return value;
        }

        uint8_t Mmio::read8(uintptr_t offset)
        {
            uint8_t value;
            checkError(mmio_read8(&m_mmio, offset, &value));
            return value;
        }

        std::vector<uint8_t> Mmio::read(uintptr_t offset, size_t len)
        {
            std::vector<uint8_t> buf(static_cast<int>(len));
            checkError(mmio_read(&m_mmio, offset, buf.data(), len));
            return buf;
        }

        void Mmio::write32(uintptr_t offset, uint32_t value)
        {
            checkError(mmio_write32(&m_mmio, offset, value));
        }

        void Mmio::write16(uintptr_t offset, uint16_t value)
        {
            checkError(mmio_write16(&m_mmio, offset, value));
        }

        void Mmio::write8(uintptr_t offset, uint8_t value)
        {
            checkError(mmio_write8(&m_mmio, offset, value));
        }

        void Mmio::write(uintptr_t offset, std::vector<uint8_t> &buf)
        {
            checkError(mmio_write(&m_mmio, offset, buf.data(), buf.size()));
        }

        void Mmio::close()
        {
            checkError(mmio_close(&m_mmio));
        }

        uintptr_t Mmio::base()
        {
            return mmio_base(&m_mmio);
        }

        size_t Mmio::size()
        {
            return mmio_size(&m_mmio);
        }

        std::string Mmio::toString(size_t len)
        {
            char *cstr = new char[len];
            mmio_tostring(&m_mmio, cstr, len);
            std::string ret(cstr);
            delete [] cstr;
            return ret;
        }

        void Mmio::checkError(int err_code)
        {
            switch(err_code)
            {
                case MMIO_ERROR_ARG: throw MmioArgError(mmio_errmsg(&m_mmio));   break;
                case MMIO_ERROR_OPEN: throw MmioOpenError(mmio_errmsg(&m_mmio));  break;
                case MMIO_ERROR_MAP: throw MmioMapError(mmio_errmsg(&m_mmio));   break;
                case MMIO_ERROR_CLOSE: throw MmioCloseError(mmio_errmsg(&m_mmio)); break;
                case MMIO_ERROR_UNMAP: throw MmioUnmapError(mmio_errmsg(&m_mmio)); break;
                default: /* no defined error */ break;
            }
        }
    }

}
