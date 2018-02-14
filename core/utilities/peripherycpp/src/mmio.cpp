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
            uint8_t *buf;
            int err_num = mmio_read(&mmio, offset, buf, len);
            // Add error checking
            int i = 0;
            std::vector<uint8_t> value;
            while (i < len)
            {
                value.push_back(*buf);
                ++buf;
                i++;
            }
            return value;
        }

        void Mmio::write32(uintptr_t offset, uint32_t value)
        {

        }

        void Mmio::write16(uintptr_t offset, uint16_t value)
        {

        }

        void Mmio::write8(uintptr_t offset, uint8_t value)
        {

        }

        void Mmio::write(uintptr_t offset, std::vector<uint8_t> &buf)
        {

        }

        void Mmio::close()
        {

        }

        uintptr_t Mmio::base()
        {

        }

        size_t size()
        {

        }

        std::string toString()
        {

        }

    }

}
