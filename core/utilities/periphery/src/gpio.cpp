#include "gpio.hpp"

namespace rip
{

    namespace periphery
    {

        void gpio::open(unsigned int pin, int direction)
        {
            gpio_direction_t dir;
            switch(direction)
            {
                case 0: dir = GPIO_DIR_IN; break;
                case 1: dir = GPIO_DIR_OUT; break;
                case 2: dir = GPIO_DIR_OUT_LOW; break;
                case 3: dir = GPIO_DIR_OUT_HIGH; break;
                case 4: dir = GPIO_DIR_PRESERVE; break;
                default: return//Error throw
            }
            int err_code = gpio_open(&gpio, pin, dir);
            return;
        }

        bool gpio::read()
        {
            bool *val;
            int err_code = gpio_read(&gpio, val);
            return *val;
        }

        void gpio::write(bool value)
        {
            int err_code = gpio_write(&gpio, value);
            return;
        }

        int gpio::poll(int timeout_ms)
        {
            int pollnum = gpio_poll(&gpio, timeout_ms);
            if (pollnum == 1 || pollnum == 0)
            {
                return pollnum;
            }
            else
            {
                // Add error checking later
                return -1;
            }
        }

        void gpio::close()
        {
            int err_code = gpio_close(&gpio);
            return;
        }

        bool gpio::supports_interupts()
        {
            bool *support;
            int err_code = gpio_supports_interupts(&gpio, support);
            return *support;
        }

        int gpio::get_direction()
        {
            gpio_direction_t *dir;
            int err_code = gpio_get_direction(&gpio, dir);
            int dirnum;
            switch(*dir)
            {
                case GPIO_DIR_IN: dirnum = 0; break;
                case GPIO_DIR_OUT: dirnum = 1; break;
                case GPIO_DIR_OUT_LOW: dirnum = 2; break;
                case GPIO_DIR_OUT_HIGH: dirnum = 3; break;
                case GPIO_DIR_PRESERVE: dirnum = 4; break;
                default: return//Error throw
            }
            return dirnum;
        }

        int gpio::get_edge()
        {
            gpio_edge_t *edge;
            int err_code = gpio_get_edge(&gpio, edge);
            int edgenum;
            switch(*dir)
            {
                case GPIO_EDGE_NONE: edgenum = 0; break;
                case GPIO_EDGE_RISING: edgenum = 1; break;
                case GPIO_EDGE_FALLING: edgenum = 2; break;
                case GPIO_EDGE_BOTH: edgenum = 3; break;
                default: return//Error throw
            }
            return edgenum;
        }

        void gpio::set_direction(int direction)
        {
            gpio_direction_t dir;
            switch(direction)
            {
                case 0: dir = GPIO_DIR_IN; break;
                case 1: dir = GPIO_DIR_OUT; break;
                case 2: dir = GPIO_DIR_OUT_LOW; break;
                case 3: dir = GPIO_DIR_OUT_HIGH; break;
                case 4: dir = GPIO_DIR_PRESERVE; break;
                default: return//Error throw
            }
            int err_code = gpio_set_direction(&gpio, dir);
            // Additional error checking
            return;
        }

        void gpio::set_edge(int edge)
        {
            gpio_edge_t ed;
            switch(edge)
            {
                case 0: ed = GPIO_EDGE_NONE; break;
                case 1: ed = GPIO_EDGE_RISING; break;
                case 2: ed = GPIO_EDGE_FALLING; break;
                case 3: ed = GPIO_EDGE_BOTH; break;
                default: return//Error throw
            }
            int err_code = gpio_set_edge(&gpio, ed);
            // Additional error checking
            return;
        }

        unsigned int gpio::pin()
        {
            unsigned int pinnum;
            pinnum = gpio_pin(&gpio);
            return pinnum;
        }

        int gpio::fd()
        {
            int filedescriptor = gpio_fd(&gpio);
            return filedescriptor;
        }

        std::string gpio::tostring(size_t len)
        {
            char *cstr;
            int err_num = gpio_tostring(&gpio, cstr, len);
            std::string str = cstr;
            return str;
        }

    }

}
