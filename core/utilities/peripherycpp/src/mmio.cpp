#include "mmio.hpp"
#include <cstdint>
#include <vector>

namespace rip
{

    namespace periphery
    {

        void Mmio::open(uintptr_t base, size_t size)
        {
            int err_num = mmio_open(&mmio, base, size);
            // Add error checking
            return;
        }

        // void *ptr()

        uint32_t Mmio::read32(uintptr_t offset)
        {
            uint32_t *value;
            int err_num = mmio_read32(&mmio, offset, value);
            // Add error checking
            return *value;
        }

        uint16_t Mmio::read16(uintptr_t offset)
        {
            uint16_t *value;
            int err_num = mmio_read16(&mmio, offset, value);
            // Add error checking
            return *value;
        }

        uint8_t Mmio::read8(uintptr_t offset)
        {
            uint8_t *value;
            int err_num = mmio_read8(&mmio, offset, value);
            // Add error checking
            return *value;
        }

        std::vector<uint8_t> Mmio::read(uintptr_t offset, size_t len)
        {
            std::vector<uint8_t> buf(static_cast<int>(len));
            int err_num = mmio_read(&mmio, offset, &buf[0], len);
            // Add error checking
            return buf;
        }

        void Mmio::write32(uintptr_t offset, uint32_t value)
        {
            int err_num = mmio_write32(&mmio, offset, value);
            // Add error checking
            return;
        }

        void Mmio::write16(uintptr_t offset, uint16_t value)
        {
            int err_num = mmio_write16(&mmio, offset, value);
            // Add error checking
            return;
        }

        void Mmio::write8(uintptr_t offset, uint8_t value)
        {
            int err_num = mmio_write8(&mmio, offset, value);
            // Add error checking
            return;
        }

        void Mmio::write(uintptr_t offset, std::vector<uint8_t> &buf)
        {
            int err_num = mmio_write(&mmio, offset, &buf[0], buf.size());
            // Add error checking
            return;
        }

        void Mmio::close()
        {
            int err_num = mmio_close(&mmio);
            // Add error checking
            return;
        }

        uintptr_t Mmio::base()
        {
            return mmio_base(&mmio);
        }

        size_t size()
        {
            return mmio_size(&mmio);
        }

        std::string toString(size_t len)
        {
            char *cstr;
            int err_num = mmio_tostring(&mmio, cstr, len);
            // Add error checking
            string str = cstr;
            return str;
        }

    }

}
