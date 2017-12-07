#ifndef SPI_HPP
#define SPI_HPP

#include <vector>
#include <string>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include "exceptions.hpp"

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace spi {
                /**
                 * Class for managing sending and receiving data through the SPI-bus
                 */
                class Spi
                {
                public:
                    Spi();

                    Spi(const std::string& device);
                    ~Spi();

                    //add description
                    void setCS(int cs);
                    /**
                     * [setMode description]
                     * @param mode [description]
                     */
                    void setMode(uint8_t mode);

                    /**
                     * [getMode description]
                     * @return [description]
                     */
                    uint8_t getMode();

                    /**
                     * [setLSB description]
                     * @param lsb [description]
                     */
                    void setLSB(uint8_t lsb);

                    /**
                     * [getLSB description]
                     * @return [description]
                     */
                    uint8_t getLSB();

                    /**
                     * [setLength description]
                     * @param length [description]
                     */
                    void setLength(uint8_t length);

                    /**
                     * [getLength description]
                     * @return [description]
                     */
                    uint8_t getLength();

                    /**
                     * [setSpeed description]
                     * @param speed [description]
                     */
                    void setSpeed(uint8_t speed);

                    /**
                     * [getSpeed description]
                     * @return [description]
                     */
                    uint8_t getSpeed();

                    /**
                     * Read \p length bytes from the other device
                     * @param length Number of bytes to read
                     * @returns The specified number of bytes received from the other device
                     */
                    std::vector<uint8_t> read(size_t length = 1);


                    /**
                     * Sends a message to the other device
                     * @param message The message to send
                     */
                    void write(std::vector<uint8_t> message);
                private:
                    void open();
                    void close();
                    std::string m_device;
                    const std::string kDefaultDevice = "/dev/ttyS0";//assuming use of mini UART
                    int m_fd;
                };
            } // spi
        } // pins
    } // utilities
} // rip

#endif // SPI_HPP
