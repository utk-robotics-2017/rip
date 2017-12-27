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
    {"ticks_per_rev", 465.0},{"wheel_radius", 3.625}, {"device", "/dev/ttyAMA0"},
    {"baudrate", 115200}};
    nlohmann::json config2 = config1;
    config2["address"]=0x81;
    int64_t d;
    rip::utilities::units::Distance d2;
    rip::utilities::units::Current c;
    rip::utilities::units::Velocity speed;
    MotorDynamics dynamics;
    Roboclaw testClawLeft(config2);
    Roboclaw testClawRight(config1);
    std::vector<Roboclaw*> claws = {&testClawLeft, &testClawRight};
    //std::cout<< "got here";
    //setBaudRate(115200);
    //testClawRight.setPort("/dev/ttyS0");

    diag::Diag switchStatements(claws);
    //switchStatements.start();
    d=testClawRight.readEncoderRaw(Roboclaw::Motor::kM1);
    std::cout << "d right: " << d << std::endl;
d=testClawLeft.readEncoderRaw(Roboclaw::Motor::kM1);
    std::cout << "d left: " << d << std::endl;
    std::cout << "version: " << testClawRight.readVersion() << std::endl;
    d2 = testClawRight.readEncoder(Roboclaw::Motor::kM1);
    std::cout << "d2: " << d2 << std::endl;
    //testClawLeft.resetEncoders();
    /*
    c=testClawLeft.readMaxCurrent(Roboclaw::Motor::kM1);
    std::cout << "c: " << d2 << std::endl;*/
    d2 = 1000 * rip::utilities::units::m;
    speed = rip::utilities::units::foot / rip::utilities::units::s;
    speed = speed * 1000;
    dynamics.setDistance(d2);
    dynamics.setSpeed(speed);

    testClawRight.setDynamics(dynamics);
    testClawLeft.setDynamics(dynamics);
    std::cout << "Roboclaw Hardware Testing" << std::endl;
    return 0;
}
