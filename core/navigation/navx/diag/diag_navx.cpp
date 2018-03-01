#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <navx/navx.hpp>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>

volatile sig_atomic_t sflag = 0;
using namespace rip::navigation::navx;
void handle_sig(int sig)
{
    sflag = 1;
}


int main(int argc, char *argv[]) {
    std::cout << "Program Executing\n";
    signal(SIGINT, handle_sig);

    NavX com = NavX("/dev/ttyACM0");

    printf("Initializing\n\n");


    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Pitch  |  Roll  |  Yaw  |  X-Accel  | Y-Accel  |  Z-Accel  |  Time  |" << std::endl;

    while(true)
    {
        std::cout << std::fixed << std::setprecision(2) << com.getPitch() << "      " << com.getRoll();
        std::cout << "   " << com.getAngle() << "     " <<com.getWorldLinearAccelX().to(units::AccelerationOfGravity) << "     ";
        std::cout << com.getWorldLinearAccelY().to(units::AccelerationOfGravity) << "       " << com.getWorldLinearAccelZ().to(units::AccelerationOfGravity) << "      ";
        std::cout << com.getLastSensorTimestamp() << "      " << '\r' << std::flush;
        //std::this_thread::sleep_for(std::chrono::milliseconds(125));
        if(sflag)
        {
            sflag = 0;
            com.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        }
    }
    printf("\nExit Caught... Closing device.\n");

    return 0;
}
