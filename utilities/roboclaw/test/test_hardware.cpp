#include "roboclaw.hpp"
/*
Interactive diagnostic tests, specifically for the Roboclaw's on the demo bot (for now)
DO NOT run without physical access, and powered on via battery.
Killing this program will not save you from the malice of motors.
*/

//goes to menu for interactive tests.
int main()
{
    /*
    nlohmann::json config1={{"address", 0x80},{"timeout", 1000},
    {"ticks_per_rev", 360.0},{"wheel_radius", 40}};
    nlohmann::json config2 = config1;
    config2["address"]=0x81;

    Roboclaw testClawLeft(config2), testClawRight(config1);

    testClawRight.setBaudrate(38400);
    testClawRight.setPort("/dev/ttyAMA0");
    //testClawRight.setsimTimeout(1000);
    testClawRight.open();
    if(testClawRight.isOpen())
    {
        testClawRight.readVersion();
    }

    std::cout << "Roboclaw Hardware Testing" << std::endl;
    */
}
