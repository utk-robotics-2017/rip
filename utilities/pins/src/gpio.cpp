#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

#include <spdlog/fmt/fmt.h>

#include "exceptions.hpp"
#include "gpio.hpp"

std::ostream& operator<<(std::ostream& estream, const rip::utilities::pins::gpio::DigitalPinValue& pinvalue) {
    if (pinvalue == rip::utilities::pins::gpio::DigitalPinValue::kLow) {
        estream << "0";
    }
    else if (pinvalue == rip::utilities::pins::gpio::DigitalPinValue::kHigh) {
        estream << "1";
    }

    return estream;
}

namespace rip
{
    namespace utilities
    {
        namespace pins
        {
            namespace gpio
            {
                void Gpio::setPinMode(uint8_t pin, DigitalPinMode mode)
                {
                    std::ofstream file ("/sys/class/gpio/export", std::ofstream::out);
                    if (file.fail() || !file.is_open())
                    {
                        throw FileAccessError("Cannot access /sys/class/gpio/export");
                    }
                    file << std::to_string(pin);
                    file.close();

                    file.open(fmt::format("/sys/class/gpio/gpio{}/direction", pin), std::ofstream::out);
                    if (file.fail() || !file.is_open())
                    {
                        throw FileAccessError(fmt::format("Cannot access /sys/class/gpio{}/direction", pin));
                    }

                    switch (mode)
                    {
                        case DigitalPinMode::kInput:
                            file << "in";
                            break;
                        case DigitalPinMode::kOutput:
                            file << "out";
                            break;
                    }
                    file.close();
                }

                void Gpio::digitalWrite(uint8_t pin, DigitalPinValue value)
                {
                    std::ofstream file(fmt::format("/sys/class/gpio/gpio{}/value", pin));
                    if (file.fail() || !file.is_open())
                    {
                        throw FileAccessError(fmt::format("Cannot access /sys/class/gpio/gpio{}/value", pin));
                    }

                    file << value; // look at the overloaded operator for DigitalPinValue
                    file.close();
                }

                DigitalPinValue Gpio::digitalRead(uint8_t pin)
                {
                    std::ifstream file(fmt::format("/sys/class/gpio/gpio{}/value", pin));
                    if (file.fail() || !file.is_open())
                    {
                        throw FileAccessError(fmt::format("Cannot open /sys/class/gpio/gpio{}/value for reading", pin));
                    }

                    std::string value;
                    file >> value;
                    file.close();
                    if (value.size() != 1)
                    {
                        throw DigitalReadError("Bad size of read from /s/c/g/X/value descriptor.");
                    }
                    return static_cast<DigitalPinValue>(value[0] - '0');
                }
            } // gpio
        } // pins
    } // utilities
} // rip
