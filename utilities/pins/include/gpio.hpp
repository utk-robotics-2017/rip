#ifndef GPIO_HPP
#define GPIO_HPP

namespace rip
{
    namespace pins
    {
        namespace gpio
        {

            enum class DigitalPinValue
            {
                kLow = 0,
                kHigh = 1
            };

            enum class DigitalPinDirection
            {
                kOut,
                kIn
            };

            /**
             *
             */
            void setPinMode(uint8_t pin, DigitalPinDirection direction);

            /**
             * [digitalWrite description]
             * @param pin   [description]
             * @param value [description]
             */
            void digitalWrite(uint8_t pin, DigitalPinValue value);

            /**
             * [digitalRead description]
             * @param  pin [description]
             * @return     [description]
             */
            DigitalPinValue digitalRead(uint8_t pin);
        };
    } // pins
} // rip

#endif // GPIO_HPP