#ifndef _PERIPHERY_GPIO_HPP
#define _PERIPHERY_GPIO_HPP

extern "C"
{
    #include "gpio.h"
}
#include "peripherycpp/exceptions.hpp"
#include <misc/logger.hpp>
#include <fmt/format.h>

namespace rip
{

    namespace peripherycpp
    {

        class Gpio
        {
            public:
                /**
                 * default constructor
                 */
                Gpio();
                /**
                 * constructor that opens a pin for reading
                 * @param pin       the Linux GPIO pin number for the pin being opened.
                 * @param direction direction  an integer corresponding to the direction the
                 *  GPIO pin is being opened (it will be converted to an enum).
                 */
                Gpio(unsigned int pin, int direction);
                /**
                 * open
                 * @param pin  the Linux GPIO pin number for the pin being opened.
                 * @param direction  an integer corresponding to the direction the GPIO pin is being
                 *  opened (it will be converted to an enum).
                 * @brief  Open the sysfs GPIO corresponding to the specified pin, with the
                 * specified direction.
                 */
                void open(unsigned int pin, int direction);

                /**
                 * read
                 * @brief  Read the state of the GPIO and return it.
                 * @return  a bool describing the state of the GPIO.
                 */
                bool read();

                /**
                 * write
                 * @param value  the value that will be used to set the state of the GPIO.
                 * @brief  Set the state of the GPIO to value.
                 */
                void write(bool value);

                /**
                 * poll
                 * @param timeout_ms  what it does depends on the value. If it is positive, it
                 * is a timeout in milliseconds. If 0, it is a non-blocking poll. If negative,
                 * it is a blocking poll.
                 * @brief  Poll a GPIO for the edge event configured with gpio_set_edge.
                 * @return  1 on success, 0 on timeout
                 */
                int poll(int timeout_ms);

                /**
                 * close
                 * @brief  Close the sysfs GPIO.
                 */
                void close();

                /**
                 * supportsInterrupts
                 * @brief Query a GPIO for edge interrupt support.
                 * @return  a bool stating whether the GPIO pin supports interrupts
                 */
                bool supportsInterrupts();

                /**
                 * getDirection
                 * @brief  Query the configured direction of the GPIO.
                 * @return  the direction, coded as an int of range [0, 4].
                 */
                int getDirection();

                /**
                 * getEdge
                 * @brief  Query the configured edge of the GPIO.
                 * @return  the edge, coded as an int of range [0, 3]
                 */
                int getEdge();

                /**
                 * setDirection
                 * @param direction  the direction to be used in setting the GPIO, coded as an int of range [0. 4].
                 * @brief  Set the direction of the GPIO.
                 */
                void setDirection(int direction);

                /**
                 * setEdge
                 * @param edge  the edge to be used in setting the GPIO, coded as an int of range [0, 3].
                 * @brief  Set the interrupt edge of the GPIO.
                 */
                void setEdge(int edge);

                /**
                 * pin
                 * @brief  Return the pin the GPIO handle was opened with.
                 * @return  the pin the GPIO handle was opened with.
                 */
                unsigned int pin();

                /**
                 * fd
                 * @brief  Return the file descriptor (for the underlying sysfs GPIO "value" file) of the GPIO handle.
                 * @return  the file descriptor of the GPIO handle.
                 */
                int fd();

                /**
                 * toString
                 * @param len  the length of the returned string.
                 * @brief  Return a string representation of the GPIO handle.
                 * @return  a string representation of the GPIO handle.
                 */
                std::string toString(size_t len);

            private:
                /**
                 * checkError
                 * @param err_code An int error code from Periphery GPIO
                 * @brief Acts as a error handler for the GPIO class
                 */
                void checkError(int err_code);

                gpio_t m_gpio;
        };

    }

}

#endif
