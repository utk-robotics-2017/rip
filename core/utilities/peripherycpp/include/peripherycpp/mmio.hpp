#ifndef _PERIPHERY_MMIO_HPP
#define _PERIPHERY_MMIO_HPP

extern "C"
{
    #include "mmio.h"
}
#include "peripherycpp/exceptions.hpp"
#include <cstdint>
#include <vector>

namespace rip
{

    namespace peripherycpp
    {
        class Mmio
        {
            public:

                void open(uintptr_t base, size_t size);

                //void *ptr();

                uint32_t read32(uintptr_t offset);

                uint16_t read16(uintptr_t offset);

                uint8_t read8(uintptr_t offset);

                std::vector<uint8_t> read(uintptr_t offset, size_t len);

                void write32(uintptr_t offset, uint32_t value);

                void write16(uintptr_t offset, uint16_t value);

                void write8(uintptr_t offset, uint8_t value);

                void write(uintptr_t offset, std::vector<uint8_t> &buf);

                void close();

                uintptr_t base();

                size_t size();

                std::string toString(size_t len);

            private:

                mmio_t mmio;

        };
    }
}

#endif
