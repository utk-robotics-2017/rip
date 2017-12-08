#include <roboclaw.hpp>
#include <iostream>
#include <serial/serial.h>
#include <inttypes.h>
using namespace rip::utilities::roboclaw;
/*
Interactive diagnostic tests, specifically for the Roboclaw's on the demo bot (for now)
DO NOT run without physical access, and powered on via battery.
Killing this program will not save you from the malice of motors.
*/
//goes to menu for interactive tests.


int main(int argc, char **argv)
{

    nlohmann::json config1={{"address", 0x80},{"timeout", 1000},
    {"ticks_per_rev", 360.0},{"wheel_radius", 40}};
    nlohmann::json config2 = config1;
    config2["address"]=0x81;
    int64_t d;
    //Roboclaw testClawLeft(config2);
    Roboclaw testClawRight(config1);

    testClawRight.setBaudRate(115200);
    testClawRight.setPort("/dev/ttyS0");


    //d=testClawRight.readEncoderRaw(Roboclaw::Motor::kM1);
    std::cout << "version: " << testClawRight.readVersion() << std::endl;

    std::cout << "Roboclaw Hardware Testing" << std::endl;
    return 0;
}
