#include <stdint.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "exceptions.hpp"
#include "i2c.hpp"
#include "fmt/format.h"

#include <linux/i2c-dev.h>

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace i2c
            {
                I2C::I2C()
                    : I2C(kDefaultDevice, -1)
                {}

                I2C::I2C(int address)
                    : I2C(kDefaultDevice, address)
                {}

                I2C::I2C(const std::string& device, int address)
                    : m_device(device)
                    , m_slave_address(address)
                    , m_fd(0)
                {
                    open();
                }

                I2C::~I2C()
                {
                    close();
                }

                void I2C::close()
                {
                    if (m_fd)
                    {
                        ::close(m_fd);
                        m_fd = 0;
                    }
                }

                int I2C::open()
                {
                    if (m_fd)
                    {
                        close();
                        m_fd = 0;
                    }

                    if (m_slave_address == -1)
                    {
                        throw SlaveAddressNotSet();
                    }

                    if ((m_fd = ::open(m_device.c_str(), I2C_RDWR)) < 0)
                    {
                        throw FileAccessError(fmt::format("Error opening {}\n", m_device));
                    }

                    if (::ioctl(m_fd, I2C_SLAVE, m_slave_address) < 0)
                    {
                        throw BadAddressError(fmt::format("Error with address {}\n", m_slave_address));
                    }
                }

                void I2C::setAddress(int address)
                {
                    m_slave_address = address;
                    open();
                }

                int I2C::getAddress()
                {
                    return m_slave_address;
                }

                void I2C::setDevice(const std::string& device)
                {
                    m_device = device;
                    open();
                }

                std::vector<uint8_t> I2C::read(size_t length)
                {
                    if (length < 1)
                    {
                        throw I2CReadError("Length must be 1 or greater");
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    uint8_t* buffer = new uint8_t[length];
                    if (::read(m_fd, buffer, length) != length)
                    {
                        throw I2CReadError(fmt::format("I2C read error. Address: {} Device: {}\n", m_slave_address, m_device));
                    }

                    return std::vector<uint8_t>(buffer, buffer + length);
                }

                std::vector<uint8_t> I2C::read(uint8_t register_address, size_t length)
                {
                    if (length < 1)
                    {
                        throw I2CReadError("Length must be 1 or greater");
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    if (::write(m_fd, &register_address, 1) != 1)
                    {
                        throw I2CWriteError("I2C write error\n");
                    }

                    uint8_t* buffer = new uint8_t[length];
                    if (::read(m_fd, buffer, length) != length)
                    {
                        throw I2CReadError(fmt::format("I2C read error. Address: {} Device: {}\n", m_slave_address, m_device));
                    }

                    return std::vector<uint8_t>(buffer, buffer + length);
                }

                void I2C::write(uint8_t register_address, std::vector<uint8_t> message)
                {

                    size_t length = message.size();
                    if (length < 1)
                    {
                        return;
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    message.insert(message.begin(), 0, register_address);
                    //message.insert(0, register_address);

                    // The spec guarantees that vectors store their elements contiguously
                    // http://www.open-std.org/jtc1/sc22/wg21/docs/lwg-defects.html#69
                    if (::write(m_fd, &message[0], length) != length)
                    {
                        throw I2CWriteError("Write returned non-zero on call");
                    }
                }

                void I2C::write(std::vector<uint8_t> message)
                {

                    size_t length = message.size();
                    if (length < 1)
                    {
                        return;
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    // The spec guarantees that vectors store their elements contiguously
                    // http://www.open-std.org/jtc1/sc22/wg21/docs/lwg-defects.html#69
                    if (::write(m_fd, &message[0], length) != length)
                    {
                        throw I2CWriteError("Write returned non-zero exit code");
                    }
                }
            } // i2c
        } // pins
    } // utilities
} // rip
