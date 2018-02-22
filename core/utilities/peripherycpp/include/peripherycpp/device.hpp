#ifndef _PERIPHERY_DEVICE_HPP
#define _PERIPHERY_DEVICE_HPP

#include <cstdint>
#include <vector>
#include <string>

namespace rip
{

    namespace peripherycpp
    {
        
        class Device
        {
            public:

                //Gpio
                void open(unsigned int pin, int direction) = 0;

                //I2C
                void open(const std::string path) = 0;

                //MMio
                void open(uintptr_t base, size_t size) = 0;

                //Serial
                void open(std::string device, unsigned int baudrate) = 0;

                //Serial
                void open(std::string device, unsigned int baudrate, unsigned int databits,
                          int parity, unsigned int stopbits, bool xonxoff, bool rtscts) = 0;

                //Spi
                void open(const std::string path, unsigned int mode, uint32_t max_speed) = 0;

                //Spi
                void open(const std::string path, unsigned int mode, uint32_t max_speed,
                          int bit_order, uint8_t bits_per_word, uint8_t extra_flags) = 0;

                //Gpio
                bool read() = 0;

                //MMio
                std::vector<uint8_t> read(uintptr_t offset, size_t len) = 0;

                //Serial
                std::vector<uint8_t> read(size_t len, int timeout_ms) = 0;

                //Gpio
                void write(bool value) = 0;

                //Mmio
                void write(uintptr_t offset, std::string &buf) = 0;

                //Serial
                void write(std::string data) = 0;

                void close() = 0;

                //Gpio
                int poll(int timeout_ms) = 0;

                //Serial
                bool poll(int timeout_ms) = 0;

                int fd() = 0;

                std::string toString(int len) = 0;

        };

    }

}
