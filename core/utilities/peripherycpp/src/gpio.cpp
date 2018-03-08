#include "peripherycpp/gpio.hpp"

namespace rip
{
    namespace peripherycpp
    {
        Gpio::Gpio()
        {}

        Gpio::Gpio(unsigned int pin, int direction)
        {
            open(pin, direction);
        }
        void Gpio::open(unsigned int pin, int direction)
        {
            gpio_direction_t dir;
            switch(direction)
            {
                case 0: dir = GPIO_DIR_IN; break;
                case 1: dir = GPIO_DIR_OUT; break;
                case 2: dir = GPIO_DIR_OUT_LOW; break;
                case 3: dir = GPIO_DIR_OUT_HIGH; break;
                case 4: dir = GPIO_DIR_PRESERVE; break;
                default: throw GpioArgError("direction must be in the range [0, 4].");
            }
            checkError(gpio_open(&m_gpio, pin, dir));
            misc::Logger::getInstance()->debug(fmt::format("Pin {} successfully open", pin));
            return;
        }

        bool Gpio::read()
        {
            bool val;
            checkError(gpio_read(&m_gpio, &val));
            return val;
        }

        void Gpio::write(bool value)
        {
            checkError(gpio_write(&m_gpio, value));
            return;
        }

        int Gpio::poll(int timeout_ms)
        {
            int pollnum = gpio_poll(&m_gpio, timeout_ms);
            if (pollnum == 1 || pollnum == 0)
            {
                return pollnum;
            }
            else
            {
                checkError(pollnum);
                return -1;
            }
        }

        void Gpio::close()
        {
            checkError(gpio_close(&m_gpio));
        }

        bool Gpio::supportsInterrupts()
        {
            bool support;
            checkError(gpio_supports_interrupts(&m_gpio, &support));
            return support;
        }

        int Gpio::getDirection()
        {
            gpio_direction_t dir;
            int dirnum = 4; // by default preserve direction

            checkError(gpio_get_direction(&m_gpio, &dir));

            switch(dir)
            {
                case GPIO_DIR_IN: dirnum = 0; break;
                case GPIO_DIR_OUT: dirnum = 1; break;
                case GPIO_DIR_OUT_LOW: dirnum = 2; break;
                case GPIO_DIR_OUT_HIGH: dirnum = 3; break;
                case GPIO_DIR_PRESERVE: dirnum = 4; break;
                default: throw GpioGetDirectionError("Invalid enum value"); break;
            }
            return dirnum;
        }

        int Gpio::getEdge()
        {
            gpio_edge_t edge;
            int edgenum;

            checkError(gpio_get_edge(&m_gpio, &edge));

            switch(edge)
            {
                case GPIO_EDGE_NONE: edgenum = 0; break;
                case GPIO_EDGE_RISING: edgenum = 1; break;
                case GPIO_EDGE_FALLING: edgenum = 2; break;
                case GPIO_EDGE_BOTH: edgenum = 3; break;
                default: throw GpioGetEdgeError("Invalid enum value"); break;
            }
            return edgenum;
        }

        void Gpio::setDirection(int direction)
        {
            gpio_direction_t dir;
            switch(direction)
            {
                case 0: dir = GPIO_DIR_IN; break;
                case 1: dir = GPIO_DIR_OUT; break;
                case 2: dir = GPIO_DIR_OUT_LOW; break;
                case 3: dir = GPIO_DIR_OUT_HIGH; break;
                case 4: dir = GPIO_DIR_PRESERVE; break;
                default: throw GpioArgError("direction must be in the range [0, 4]."); break;
            }
            checkError(gpio_set_direction(&m_gpio, dir));
        }

        void Gpio::setEdge(int edge)
        {
            gpio_edge_t ed;
            switch(edge)
            {
                case 0: ed = GPIO_EDGE_NONE; break;
                case 1: ed = GPIO_EDGE_RISING; break;
                case 2: ed = GPIO_EDGE_FALLING; break;
                case 3: ed = GPIO_EDGE_BOTH; break;
                default: throw GpioArgError("edge must be in the range [0, 3].");
            }
            checkError(gpio_set_edge(&m_gpio, ed));
        }

        unsigned int Gpio::pin()
        {
            unsigned int pinnum;
            pinnum = gpio_pin(&m_gpio);
            return pinnum;
        }

        int Gpio::fd()
        {
            int filedescriptor = gpio_fd(&m_gpio);
            return filedescriptor;
        }

        std::string Gpio::toString(size_t len)
        {
            char *cstr = new char[len];
            gpio_tostring(&m_gpio, cstr, len);
            std::string ret(cstr);
            delete [] cstr;
            return ret;
        }

        void Gpio::checkError(int err_code)
        {
            switch(err_code)
            {
                case GPIO_ERROR_ARG: throw GpioArgError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_EXPORT: throw GpioExportError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_OPEN: throw GpioOpenError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_IO: throw GpioIoError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_CLOSE: throw GpioCloseError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_SET_DIRECTION:
                case GPIO_ERROR_GET_DIRECTION: throw GpioGetDirectionError(gpio_errmsg(&m_gpio)); break;
                case GPIO_ERROR_SET_EDGE:
                case GPIO_ERROR_GET_EDGE: throw GpioGetEdgeError(gpio_errmsg(&m_gpio)); break;
                default: /* no defined error */ break;
              }
        }
    }
}
