#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <fmt/format.h>
#include "exceptions.hpp"

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace serial
            {
                /**
                 * Class for managing sending and receiving data through the serial bus
                 */
                class Serial
                {
                public:
                    /**
                     * Default Constructor
                     */
                    Serial();

                    /**
                     * Constructor
                     *
                     * @param device The path to the device to talk to
                     */
                    Serial(const std::string& device);

                    /**
                     * Constructor
                     *
                     * @param device The path to the device to talk to
                     * @param baudrate The speed of the communication
                     */
                    Serial(const std::string& device, int baudrate);

                    /**
                     * Read \p length bytes from the other device
                     * @param length Number of bytes to read
                     * @returns The specified number of bytes received from the other device
                     */
                    virtual std::vector<uint8_t> read(size_t length = 1);

                    /**
                     * Sends a message to the other device
                     * @param message The message to send
                     */
                    virtual void write(std::vector<uint8_t> message);

                    /*
                     * Sets the baud rate
                     * @param The baud rate
                     */
                     void setBaudRate(int baud);
                     /*
                      * Sets the port/device
                      * @param device The port/device name.
                      */
                     void setPort(std::string device);

                private:
                    void open();
                    void close();
                    struct termios m_config;
                    std::string m_device;
                    long m_baudrate;
                    int m_fd;
                    const std::string kDefaultDevice = "/dev/ttyS0";
                    const int kDefaultBaudRate = 115200;


                };
            } //serial
        } // pins
    } // utilities
} // rip

#endif
