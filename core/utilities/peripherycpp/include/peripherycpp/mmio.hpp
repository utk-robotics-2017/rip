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

                /**
                 * open
                 * @param base  the base address where the physical memory will be mapped.
                 * @param size  the size of the mapping.
                 * @brief  Map the region of physical memory at the specified base address with the specified size.
                 */
                void open(uintptr_t base, size_t size);

                //void *ptr();

                /**
                 * read32
                 * @param offset  the read offset from the base address that was used in open.
                 * @brief  Read 32-bits from mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                uint32_t read32(uintptr_t offset);

                /**
                 * read16
                 * @param offset  the read offset from the base address that was used in open.
                 * @brief  Read 16-bits from mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                uint16_t read16(uintptr_t offset);

                /**
                 * read8
                 * @param offset  the read offset from the base address that was used in open.
                 * @brief  Read 8-bits from mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                uint8_t read8(uintptr_t offset);

                /**
                 * read
                 * @param offset  the read offset from the base address that was used in open.
                 * @param len  the number of bytes to be read.
                 * @brief  Read an array of bytes from mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                std::vector<uint8_t> read(uintptr_t offset, size_t len);

                /**
                 * write32
                 * @param offset  the write offset from the base address that was used in open.
                 * @param value  the data to be written.
                 * @brief  Write 32-bits to mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                void write32(uintptr_t offset, uint32_t value);

                /**
                 * write16
                 * @param offset  the write offset from the base address that was used in open.
                 * @param value  the data to be written.
                 * @brief  Write 16-bits to mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                void write16(uintptr_t offset, uint16_t value);

                /**
                 * write8
                 * @param offset  the write offset from the base address that was used in open.
                 * @param value  the data to be written.
                 * @brief  Write 8-bits to mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                void write8(uintptr_t offset, uint8_t value);

                /**
                 * write
                 * @param offset  the write offset from the base address that was used in open.
                 * @param value  the data to be written.
                 * @brief  Write an array of bytes to mapped physical memory, starting at the specified byte offset, relative to the base address the MMIO handle was opened with.
                 */
                void write(uintptr_t offset, std::vector<uint8_t> &buf);

                /**
                 * close
                 * @brief  Unmap mapped physical memory.
                 */
                void close();

                /**
                 * base
                 * @brief  Return the base address the MMIO handle was opened with.
                 */
                uintptr_t base();

                /**
                 * size
                 * @brief  Return the size the MMIO handle was opened with.
                 */
                size_t size();

                /**
                 * toString
                 * @param len  the length of the string to be returned.
                 * @brief  Return a string representation of the MMIO handle.
                 */
                std::string toString(size_t len);

            private:

                mmio_t mmio;

        };
    }
}

#endif
