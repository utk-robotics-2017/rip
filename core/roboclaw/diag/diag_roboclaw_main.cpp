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
    nlohmann::json config1={
        {"address", 0x80},
        {"timeout", 1000},
        {"ticks_per_rev", 3591.84},
        {"wheel_radius", (2.875/2.0) * rip::units::in},
        {"device", "/dev/ttyAMA0"},
        {"baudrate", 115200}
    };
    nlohmann::json config2 = config1;
    config2["address"]=0x81;

    Roboclaw testClawLeft(config2);
    Roboclaw testClawRight(config1);
    std::vector<Roboclaw*> claws = {&testClawLeft, &testClawRight};

    diag::Diag diagTool(claws);
    diagTool.start();
    /*
    MotorDynamics dynamics, stop;
    stop.setSpeed(0);
    stop.setDistance(0);
    int64_t ticks;
    rip::utilities::units::Velocity speed;
    rip::utilities::units::Distance d2;
    testClawRight.resetEncoders();
    testClawLeft.resetEncoders();

    d2 = 2.875 * rip::utilities::units::pi * rip::utilities::units::in;
    speed = rip::utilities::units::foot / rip::utilities::units::s;
    dynamics.setDistance(d2);
    dynamics.setSpeed(speed);


    testClawRight.setDynamics(dynamics);
    testClawLeft.setDynamics(dynamics);
    testClawRight.setDynamics(stop, false);
    testClawLeft.setDynamics(stop, false);

    //std::cout << "ticks m1 left " << testClawLeft.readEncoderRaw(Roboclaw::Motor::kM1) << std::endl;
    /*
    testClawRight.resetEncoders();
    testClawLeft.resetEncoders();

    //d2 = .073025 * rip::utilities::units::pi * rip::utilities::units::m;
    speed = 10 * rip::utilities::units::foot / rip::utilities::units::s;
    dynamics.setDistance(d2);
    dynamics.setSpeed(speed);

    testClawRight.setDynamics(dynamics);
    testClawLeft.setDynamics(dynamics);
    std::cout << "ticks m1 left " << testClawLeft.readEncoderRaw(Roboclaw::Motor::kM1) << std::endl;
    */
    return 0;
}
