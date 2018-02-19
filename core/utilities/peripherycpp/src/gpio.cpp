#include "peripherycpp/gpio.hpp"

namespace rip
{
    namespace peripherycpp
    {
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
            int err_code = gpio_open(&gpio, pin, dir);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -2: throw GpioExportError(gpio_errmsg(&gpio)); break;
                case -3: throw GpioOpenError(gpio_errmsg(&gpio)); break;
                case -6: throw GpioSetDirectionError(gpio_errmsg(&gpio)); break;
                default: break;
            }
            return;
        }

        bool Gpio::read()
        {
            bool val;
            int err_code = gpio_read(&gpio, &val);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -4: throw GpioIoError(gpio_errmsg(&gpio)); break;
                default: break;
            }
            return val;
        }

        void Gpio::write(bool value)
        {
            int err_code = gpio_write(&gpio, value);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -4: throw GpioIoError(gpio_errmsg(&gpio)); break;
                default: break;
            }
            return;
        }

        int Gpio::poll(int timeout_ms)
        {
            int pollnum = gpio_poll(&gpio, timeout_ms);
            if (pollnum == 1 || pollnum == 0)
            {
                return pollnum;
            }
            else
            {
                switch(pollnum)
                {
                    case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                    case -4: throw GpioIoError(gpio_errmsg(&gpio)); break;
                    default: break;
                }
                return -1;
            }
        }

        void Gpio::close()
        {
            int err_code = gpio_close(&gpio);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -5: throw GpioCloseError(gpio_errmsg(&gpio)); break;
                default: break;
            }
            return;
        }

        bool Gpio::supportsInterrupts()
        {
            bool support;
            int err_code = gpio_supports_interrupts(&gpio, &support);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -4: throw GpioIoError(gpio_errmsg(&gpio)); break;
                default: break;
            }
            return support;
        }

        int Gpio::getDirection()
        {
            gpio_direction_t dir;
            int err_code = gpio_get_direction(&gpio, &dir);
            int dirnum = 4; // by default preserve direction

            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -7: throw GpioGetDirectionError(gpio_errmsg(&gpio)); break;
                default: break;
            }

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
            int err_code = gpio_get_edge(&gpio, &edge);

            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -9: throw GpioGetEdgeError(gpio_errmsg(&gpio)); break;
                default: /* do nothing */ break;
            }

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
            int err_code = gpio_set_direction(&gpio, dir);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -6: throw GpioSetDirectionError(gpio_errmsg(&gpio)); break;
                default: break;
            }
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
            int err_code = gpio_set_edge(&gpio, ed);
            switch(err_code)
            {
                case -1: throw GpioArgError(gpio_errmsg(&gpio)); break;
                case -8: throw GpioSetEdgeError(gpio_errmsg(&gpio)); break;
                default: break;
            }
        }

        unsigned int Gpio::pin()
        {
            unsigned int pinnum;
            pinnum = gpio_pin(&gpio);
            return pinnum;
        }

        int Gpio::fd()
        {
            int filedescriptor = gpio_fd(&gpio);
            return filedescriptor;
        }

        std::string Gpio::toString(size_t len)
        {
            char *cstr = new char[len];
            gpio_tostring(&gpio, cstr, len);
            return std::string(cstr);
        }
    }
}
