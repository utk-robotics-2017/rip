#include <iostream>

#include "arduino_gen.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./arduino_gen <config_file>" << std::endl;
    }

    std::unique_ptr<rip::arduinogen::ArduinoGen> ag = std::unique_ptr<rip::arduinogen::ArduinoGen>(new rip::arduinogen::ArduinoGen("mega", "/", "test/data/arduino_gen", true));

    ag->readConfig(argv[1], false);

    std::cout << ag->getArduinoCode() << std::endl;
}
