#include <roboclaw.hpp>
#include <iostream>
#include <inttypes.h>
#include <serial.h>
#include <units.hpp>
#include "diag_roboclaw.hpp"

using namespace rip::utilities::roboclaw;
/*
Interactive diagnostic tests, specifically for the Roboclaw's on the demo bot (for now)
DO NOT run without physical access, and powered on via battery.
Killing this program will not save you from the malice of motors.
*/

int main(int argc, char **argv)
{
    //make serial stuff
    nlohmann::json config1={{"address", 0x80},{"timeout", 1000},
    {"ticks_per_rev", 3591.84},{"wheel_radius", 3.625 * rip::utilities::units::cm}, {"device", "/dev/ttyAMA0"},
    {"baudrate", 115200}};
    nlohmann::json config2 = config1;
    config2["address"]=0x81;

    Roboclaw testClawLeft(config2);
    Roboclaw testClawRight(config1);
    std::vector<Roboclaw*> claws = {&testClawLeft, &testClawRight};
    
    diag::Diag diagTool(claws);
    diagTool.start();
    return 0;
}
