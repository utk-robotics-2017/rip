#include <roboclaw.hpp>
#include <iostream>
#include <inttypes.h>
#include <serial.h>
#include <units.hpp>
/*
Interactive diagnostic tests, specifically for the Roboclaw's on the demo bot (for now)
DO NOT run without physical access, and powered on via battery.
Killing this program will not save you from the malice of motors.
*/

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace diag
            {
                class Diag
                {
                public:
                    /*
                    Constructor - takes vector of memory addresses of roboclaw's
                    */
                    Diag(std::vector<Roboclaw*> roboclaws);

                    /*
                    Main menu - switch statement, organizes submenus. Some actual
                    diag tests here for those who don't fit into a category well and
                    might be frequently used
                    */
                    void mainMenu();
                    /*
                    Menu that gives access to a list of battery voltage commands that can be run
                    for diagnostic purposes.
                    */
                    void voltageMenu();
                    /*
                    Attempts to read the status flag's of the roboclaw.
                    */
                    void clawStatus();
                    /*
                    Attempt's to read the version of the roboclaw.
                    */
                    void clawVersion();
                    /*
                    Menu that gives access to a list of basic driving commands that can be run
                    for diagnostic purposes.
                    */
                    void simpleDriveMenu();
                    /*
                    Menu that gives access to a list of advanced driving commands that can be run
                    for diagnostic purposes.
                    */
                    void advDriveMenu();
                    /*
                    Menu that gives access to a list of encoder commands that can be run
                    for diagnostic purposes.
                    */
                    void encoderMenu();
                    /*
                    Menu that gives access to a list of battery current commands that can be run
                    for diagnostic purposes.
                    */
                    void currentMenu();
                    /*
                    Menu that gives access to a list of configuration commands that can be run
                    for diagnostic purposes.
                    */
                    void configMenu();
                    /*
                    Menu that gives access to a list of PID tuning commands that can be run
                    for diagnostic purposes.
                    */
                    void pidMenu();
                    /*
                    Menu that gives access to a miscellaneous list of roboclaw commands that can be run
                    for diagnostic purposes.
                    */
                    void miscMenu();
                    /*
                    Start, shows some generic warning message followed by the main "menu"
                    */
                    void start();
                    /*
                    Use if you want to stop running diagnostics on a roboclaw. takes the index
                    of the roboclaw to remove.
                    */
                    void removeClaw(int index);
                private:
                    std::vector<Roboclaw*> m_roboclaws;
                };
            }
        }
    }
}
