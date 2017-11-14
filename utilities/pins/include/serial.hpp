#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <vector>
#include <string>
#include <stdint.h>

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
                    std::vector<uint8_t> read(size_t length = 1);


                    /**
                     * Sends a message to the other device
                     * @param message The message to send
                     */
                    void write(std::vector<uint8_t> message);
                private:
                    void open();
                    void close();
                    terminos m_config;
                    std::string m_device;
                    long m_baudrate;
                    int m_fd;

                };
            } //serial
        } // pins
    } // utilities
} // rip

#endif
