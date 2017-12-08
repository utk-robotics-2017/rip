#include "serial.hpp"
#include <sys/ioctl.h>

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace serial
            {
                Serial::Serial()
                    : Serial(kDefaultDevice, kDefaultBaudRate)
                {}

                Serial::Serial(const std::string& device)
                    : Serial(device, kDefaultBaudRate)
                {}

                Serial::Serial(const std::string& device, int baudrate)
                    : m_device(device)
                    , m_baudrate(baudrate)
                    , m_fd(0)
                {
                    open();
                }

                void Serial::open()
                {
                    if (m_fd)
                    {
                        close();
                        m_fd = 0;
                    }

                    if ((m_fd = ::open(m_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
                    {
                        throw SerialOpenError(fmt::format("Error opening {}\n", m_device));
                    }

                    if (!isatty(m_fd))
                    {
                        throw SerialOpenError("Error isatty(fd)\n");
                    }

                    if (tcgetattr(m_fd, &m_config) < 0)
                    {
                        throw SerialOpenError("Error tcgetattr");
                    }

                    // Input flags - Turn off input processing, convert break to null byte,
                    // no CR to NL translation, no NL to CR translation, don't mark parity
                    // errors or breaks, no input parity check, don't strip high bit off,
                    // no XON/XOFF software flow control
                    m_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

                    // Output flags - Turn off output processing, no CR to NL translation,
                    // no NL  to CR-NL translation, no NL to CR translation, no column 0 CR
                    // suppression, no Ctrl-D suppression, no fill characters, no case mapping,
                    // no local output processing
                    // m_config.c_oflag &=  ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
                    m_config.c_oflag = 0;

                    // No line processing
                    // echo off, echo newline off, canonical mode off, extended input processing off,
                    // signal chars off
                    m_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

                    // Turn off character processing
                    // clear current char size mask, no parity checking,
                    // no output processing, force 8 bit input
                    m_config.c_cflag &= ~(CSIZE | PARENB);
                    m_config.c_cflag |= CS8;

                    // One input byte is enough to return from read()
                    // Inter-character timer off
                    m_config.c_cc[VMIN] = 1;
                    m_config.c_cc[VTIME] = 0;

                    // Communication speed (simple version, using the predefined constants)
                    switch (m_baudrate)
                    {
                        case 300:
                            if (::cfsetispeed(&m_config, B300) < 0 || ::cfsetospeed(&m_config, B300) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 300");
                            }
                            break;
                        case 600:
                            if (::cfsetispeed(&m_config, B600) < 0 || ::cfsetospeed(&m_config, B600) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 600");
                            }
                            break;
                        case 1200:
                            if (::cfsetispeed(&m_config, B1200) < 0 || ::cfsetospeed(&m_config, B1200) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 1200");
                            }
                            break;
                        case 2400:
                            if (::cfsetispeed(&m_config, B2400) < 0 || ::cfsetospeed(&m_config, B2400) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 2400");
                            }
                            break;
                        case 4800:
                            if (::cfsetispeed(&m_config, B4800) < 0 || ::cfsetospeed(&m_config, B4800) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 4800");
                            }
                            break;
                        case 9600:
                            if (::cfsetispeed(&m_config, B9600) < 0 || ::cfsetospeed(&m_config, B9600) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 9600");
                            }
                            break;
                        case 19200:
                            if (::cfsetispeed(&m_config, B19200) < 0 || ::cfsetospeed(&m_config, B19200) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 19200");
                            }
                            break;
                        case 38400:
                            if (::cfsetispeed(&m_config, B38400) < 0 || ::cfsetospeed(&m_config, B38400) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 38400");
                            }
                            break;
                        case 115200:
                            if (::cfsetispeed(&m_config, B115200) < 0 || ::cfsetospeed(&m_config, B115200) < 0)
                            {
                                throw BaudRateError("Error setting I/O Speed to 115200");
                            }
                            break;
                        default:
                            //Perhaps we should set to default here instead of throwing?
                            throw BaudRateError(fmt::format("Error: unknown baud rate {}", m_baudrate));
                    }

                    // Finally apply the configuration
                    if (::tcsetattr(m_fd, TCSAFLUSH, &m_config) < 0)
                    {
                        throw SerialOpenError("Error set attr\n");
                    }
                }

                void Serial::close()
                {
                    if (m_fd)
                    {
                        ::close(m_fd);
                        m_fd = 0;
                    }
                }

                void Serial::setBaudRate(int baud)
                {
                    m_baudrate = baud;
                    open();
                }

                void Serial::setPort(std::string device)
                {
                    m_device = device;
                    open();
                }

                std::vector<uint8_t> Serial::read(size_t length)
                {
                    if (length < 1)
                    {
                        throw SerialReadError("Length must be 1 or greater");
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    uint8_t* buffer = new uint8_t[length];
                    if (::read(m_fd, buffer, length) != length)
                    {
                        throw SerialReadError(fmt::format("Serial read error. Device: {}\n",  m_device));
                    }

                    return std::vector<uint8_t>(buffer, buffer + length);
                }

                void Serial::write(std::vector<uint8_t> message)
                {

                    size_t length = message.size();
                    if (length < 1)
                    {
                      throw SerialWriteError("Length must be 1 or greater");
                    }

                    if (!m_fd)
                    {
                        open();
                    }

                    // The spec guarantees that vectors store their elements contiguously
                    // http://www.open-std.org/jtc1/sc22/wg21/docs/lwg-defects.html#69
                    if (::write(m_fd, &message[0], length) != length)
                    {
                        throw SerialWriteError("Serial write error");
                    }
                }
            } // serial
        } // pins
    } // utilities
} // rip
