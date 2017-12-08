#include "spi.hpp"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace spi
            {
                Spi::Spi()
                    : Spi(kDefaultDevice)
                {}

                Spi::Spi(const std::string& device)
                    : m_device(device)
                {
                    open();
                }

                void Spi::open()
                {
                    if (m_fd)
                    {
                        close();
                    }

                    if ((m_fd = ::open(m_device.c_str(), O_RDWR)) < 0)
                    {
                        ::system("modprobe spi-bcm2835");
                        ::sleep(1);
                        if ((m_fd = ::open(m_device.c_str(), O_RDWR)) < 0)
                        {
                            throw SPIOpenError(fmt::format("Error opening {}\n", m_device));
                        }
                    }
                }

                Spi::~Spi()
                {
                    close();
                }

                void Spi::close()
                {
                    if (m_fd)
                    {
                        ::close(m_fd);
                        m_fd = 0;
                    }
                }

                void Spi::setCS(int cs)
                {
                    m_device = fmt::format("/dev/spidev0.{}", cs);
                    std::string temp;
                    if ((m_fd = ::open(m_device.c_str(), O_RDWR)) < 0)
                    {
                        temp = fmt::format("modprobe spi-bcm2835 cs_pin={}", cs);
                        ::system(temp.c_str());
                        ::sleep(1);
                        if ((m_fd = ::open(m_device.c_str(), O_RDWR)) < 0)
                        {
                            throw SPIWriteError("Error with setting cs");
                        }
                    }
                }

                void Spi::setMode(uint8_t mode)
                {
                    if (::ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0)
                    {
                        throw SPIWriteError(fmt::format("Error with setting mode {}", mode));
                    }
                }

                uint8_t Spi::getMode()
                {
                    __u8 mode;
                    if (::ioctl(m_fd, SPI_IOC_RD_MODE, &mode) < 0)
                    {
                        throw SPIReadError(fmt::format("Error with reading mode"));
                    }
                    return mode;
                }

                void Spi::setLSB(uint8_t lsb)
                {
                    if (::ioctl(m_fd, SPI_IOC_WR_LSB_FIRST, &lsb) < 0)
                    {
                        throw SPIWriteError(fmt::format("Error with setting lsb {}", lsb));
                    }
                }

                uint8_t Spi::getLSB()
                {
                    __u8 lsb;
                    if (::ioctl(m_fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
                    {
                        throw SPIReadError(fmt::format("Error with reading lsb"));
                    }
                    return lsb;
                }

                void Spi::setLength(uint8_t bits)
                {
                    if (::ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
                    {
                        throw SPIWriteError(fmt::format("Error with setting length {}", bits));
                    }
                }

                uint8_t Spi::getLength()
                {
                    __u8 bits;
                    if (::ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
                    {
                        throw SPIReadError(fmt::format("Error with reading length"));
                    }
                    return bits;
                }

                void Spi::setSpeed(uint8_t speed)
                {
                    if (::ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
                    {
                        throw SPIWriteError(fmt::format("Error with setting speed {}", speed));
                    }
                }

                uint8_t Spi::getSpeed()
                {
                    __u8 speed;
                    if (::ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
                    {
                        throw SPIReadError(fmt::format("Error with reading speed"));
                    }
                    return speed;
                }

                std::vector<uint8_t> Spi::read(size_t length)
                {
                    if (length < 1)
                    {
                        throw SPIReadError("Length must be 1 or greater");
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    uint8_t* buffer = new uint8_t[length];
                    if (::read(m_fd, buffer, length) != length)
                    {
                        throw SPIReadError(fmt::format("SPI read error. Device: {}\n", m_device));
                    }

                    return std::vector<uint8_t>(buffer, buffer + length);
                }

                void Spi::write(std::vector<uint8_t> message)
                {
                    size_t length = message.size();
                    if (length < 1)
                    {
                        return;
                    }

                    spi_ioc_transfer xfer;
                    xfer.tx_buf = (__u64)message[0];
                    xfer.len = length;
                    xfer.rx_buf = 0;
                    xfer.delay_usecs = 0;
                    xfer.speed_hz = 0;
                    xfer.bits_per_word = 0;
                    if (::ioctl(m_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
                    {
                        throw SPIWriteError();
                    }
                }
            }
        }
    }
}
