#include "gpio.hpp"

namespace rip
{
    namespace pins
    {
        namespace gpio
        {
            void setPinMode(uint8_t pin, DigitalPinMode mode)
            {
                std::ofstream file("/sys/clas/gpio/export");
                if (file < 0)
                {
                    throw PinModeError();
                }
                file << std::to_string(pin);
                file.close();

                file.open(fmt::format("/sys/class/gpio/gpio{}/direction", pin));
                if (file < 0)
                {
                    throw PinModeError();
                }

                switch (mode)
                {
                    case kInput:
                        file << "in";
                        break;
                    case kOutput:
                        file << "out";
                        break;
                }
                file.close();
            }

            void digitalWrite(uint8_t pin, DigitalPinValue value)
            {
                std::ofstream file(fmt::format("/sys/class/gpio/gpio{}/value", pin));
                if (file < 0)
                {
                    throw DigitalWriteError()
                }

                file << std::to_string(value);
                file.close();
            }

            DigitalPinValue digitalRead(uint8_t pin)
            {
                std::ofstream file(fmt::format("/sys/class/gpio/gpio{}/value", pin));
                if (file < 0)
                {
                    throw DigitalWriteError()
                }

                std::string value;
                file >> value;
                file.close();
                if (value.size() != 1)
                {
                    throw DigitalWriteError();
                }
                return static_cast<DigitlaPinValue>(value[0] - '0');
            }
        }
    }
} // rip