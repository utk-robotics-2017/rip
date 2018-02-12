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

        class Gpio
        {
            public:

                void open(unsigned int pin, int direction);

                bool read();

                void write(bool value);

                int poll(int timeout_ms);

                void close();

                bool supportsInterrupts();

                int getDirection();

                int getEdge();

                void setDirection(int direction);

                void setEdge(int edge);

                unsigned int pin();

                int fd();

                std::string toString(size_t len);

            private:

                gpio_t gpio;
        };

    }

}
