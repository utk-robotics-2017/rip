#ifndef I2C_HPP
#define I2C_HPP

#include <vector>
#include <string>
#include <stdint.h>

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            /**
             * @class I2C
             * @brief Class for accessing the I2C bus
             */
            class I2C
            {
            public:
                /**
                 * Default Constructor
                 */
                I2C();

                /**
                 * Constructor
                 */
                I2C(int address);

                /**
                 * Constructor
                 */
                I2C(const std::string& device, int address)

                /**
                 * Destructor
                 */
                ~I2C();

                /**
                 * [setAddress description]
                 * @param address [description]
                 */
                void setAddress(int address);

                /**
                 * [getAddress description]
                 * @return [description]
                 */
                int getAddress();

                /**
                 * [setDevice description]
                 * @param device [description]
                 */
                void setDevice(const std::string& device);

                /**
                 * Read \p length bytes from the slave
                 * @param length Number of bytes to read
                 * @returns The specified number of bytes received from the slave
                 */
                std::vector<uint8_t> read(size_t length = 1);

                /**
                 * Read \p length bytes from the slave from a specific address
                 * @param register_address The address to read from
                 * @param length Number of bytes to read
                 * @returns The specified number of bytes received from the slave
                 */
                std::vector<uint8_t> read(uint8_t register_address, size_t length);

                /**
                 * [write description]
                 * @param register_address [description]
                 * @param message          [description]
                 */
                void write(uint8_t register_address, std::vector<uint8_t> message);

                /**
                 * [write description]
                 * @param message [description]
                 */
                void write(std::vector<uint8_t> message);
            private:

                /**
                 * [open description]
                 * @return [description]
                 */
                int open();

                /**
                 * [close description]
                 */
                void close();

                std::string m_device;
                uint8_t m_fd;
                uint8_t m_slave_address;

                static constexpr kDefaultDevice = "/dev/i2c-1";
            };
        }
    }
}

#endif // I2C_HPP
