#ifndef _PERIPHERY_GPIO_HPP
#define _PERIPHERY_GPIO_HPP

extern "C"
{
    #include "gpio.h"
}

namespace rip
{

    namespace periphery
    {

        class gpio
        {
            public:

                void open(unsigned int pin, int direction);

                bool read();

                void write(bool value);

                int poll(int timeout_ms);

                void close();

                bool supports_interrupts();

                int get_direction();

                int get_edge();

                void set_direction(int direction);

                void set_edge(int edge);

                unsigned int pin();

                int fd();

                std::string tostring(size_t len);

            private:

                gpio_t gpio;
        };

    }

}
