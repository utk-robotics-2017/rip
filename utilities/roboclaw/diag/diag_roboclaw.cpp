#include "diag_roboclaw.hpp"
namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace diag
            {

                Diag::Diag(std::vector<Roboclaw*> roboclaws)
                {
                    if(roboclaws.size() > 8)
                    {
                        std::cout << std::endl << "Warning: Multi unit mode supports a maximum of 8 roboclaws per serial device" << std::endl << std::endl;
                    }
                    m_roboclaws = roboclaws;
                }

                void Diag::mainMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Diag Main Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| readVersion" << std::endl;
                        std::cout << "2| readStatus" << std::endl;
                        std::cout << "3| voltage diagnostics" << std::endl;
                        std::cout << "4| current diagnostics" << std::endl;
                        std::cout << "5| basic driving diagnostics" << std::endl;
                        std::cout << "6| advanced driving/dynamics diagnostics" << std::endl;
                        std::cout << "7| encoder diagnostics" << std::endl;
                        std::cout << "8| configuration diagnostics" << std::endl;
                        std::cout << "9| PID diagnostics" << std::endl;
                        std::cout << "10| miscellaneous" << std::endl;
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                clawVersion();
                                break;
                            case 2:
                                clawStatus();
                                break;
                            case 3:
                                voltageMenu();
                                break;
                            case 4:
                                currentMenu();
                                break;
                            case 5:
                                simpleDriveMenu();
                                break;
                            case 6:
                                advDriveMenu();
                                break;
                            case 7:
                                encoderMenu();
                                break;
                            case 8:
                                configMenu();
                                break;
                            case 9:
                                pidMenu();
                                break;
                            case 10:
                                miscMenu();
                                break;
                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::voltageMenu()
                {
                    int choice, n;
                    std::vector<std::array<units::Voltage, 2>> defaultMain;
                    std::vector<std::array<units::Voltage, 2>> defaultLogic;

                    units::Voltage v;

                    std::cout << std::endl << "NOTE: Damage can result from improper use. So don't do that" << std::endl;
                    std::cout << "Reading the current voltage values" << std::endl;

                    for(int i=0; i<m_roboclaws.size(); i++)
                    {
                        try
                        {
                            defaultMain.push_back(m_roboclaws[i]->readMinMaxMainVoltages());
                            defaultLogic.push_back(m_roboclaws[i]->readMinMaxLogicVoltages());
                        }
                        catch(const std::exception &e)
                        {
                            std::cout << "Issue reading claw " << i << " min/max voltages, " << e.what() << std::endl;
                        }
                    }
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw voltage Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "Getters (caution)" << std::endl;
                        std::cout << "1| read main battery voltage" << std::endl;
                        std::cout << "2| read logic battery voltage" << std::endl;
                        std::cout << "3| read min/max logic voltage" << std::endl;
                        std::cout << "4| read min/max main voltage" << std::endl;
                        std::cout << "Setters (caution)" << std::endl;
                        std::cout << "5| set main battery voltage min/max's" << std::endl;
                        std::cout << "6| set logic battery voltage min/max's" << std::endl;
                        std::cout << "7| set voltages back to initial values" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                            {
                                std::cout << "Exiting" << std::endl;
                                break;
                            }
                            case 1:
                            {
                                for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        v=m_roboclaws[i]->readMainBatteryVoltage();
                                        std::cout << "Claw " << i << " Main battery voltage: " << v() << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                                std::cin >> n;
                                break;
                            }
                            case 2:
                            {
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {

                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw1: " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::clawStatus()
                {
                    std::array<bool, 17> status;
                    int n;
                    std::string states[]={"Normal", "M1 OverCurrent Warning", "M2 OverCurrent Warning",
                    "E-Stop", "Temperature Error", "Temperature2 Error", "Main Battery High Error",
                    "Logic Battery High Error", "Logic Battery Low Error", "M1 Driver Fault", "M2 Driver Fault",
                    "Main Battery High Warning", "Main Battery Low Warning", "Temperature Warning",
                    "Temperature2 Warning", "M1 Home", "M2 Home"};

                    std::cout << "Reading status" << std::endl;
                    for(int i=0; i<m_roboclaws.size(); i++)
                    {
                        try
                        {
                            status = m_roboclaws[i]->readStatus();
                            for(int j=0; j<17; j++)
                            {
                                if(status[j])
                                {
                                    std::cout << states[j] << std::endl;
                                }
                            }
                        }
                        catch(const std::exception &e)
                        {
                            std::cout << "claw "<< i << "| " << e.what() << std::endl;
                        }
                    }
                    std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                    std::cin >> n;
                }

                void Diag::clawVersion()
                {
                    int n;
                    std::cout << "RoboClaw readversion" << std::endl;
                    std::cout << "Output should resemble: <USB Roboclaw 2x7a v4.1.24>" << std::endl;
                    for(int i =0; i<m_roboclaws.size(); i++)
                    {
                        try
                        {
                            std::cout << "claw " << i << " version: " << m_roboclaws[i]->readVersion() << std::endl;
                        }
                        catch(const std::exception &e)
                        {
                            std::cout << "claw: " << i <<"| " << e.what() << std::endl;
                        }
                    }
                    std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                    std::cin >> n;
                }

                void Diag::simpleDriveMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::advDriveMenu()
                {
                    int choice;
                    MotorDynamics dynamics;
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Dynamics Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| drive w/ velocity" << std::endl;
                        std::cout << "2| drive w/ velocity, acceleration" << std::endl;
                        std::cout << "3| drive w/ velocity, distance " << std::endl;
                        std::cout << "4| drive w/ velocity, distance, acceleration " << std::endl;
                        std::cout << "5| drive w/ velocity, distance " << std::endl;
                        std::cout << "6| drive w/ velocity, distance, acceleration, distance " << std::endl;
                        std::cout << "7| set velocity" << std::endl;
                        std::cout << "8| set distance" << std::endl;
                        std::cout << "9| set acceleration" << std::endl;
                        std::cout << "10| set deceleration" << std::endl;
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                try
                                {

                                }
                                catch(const std::exception &e)
                                {
                                    std::cout << "claw1: " << e.what() << std::endl;
                                }

                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::encoderMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::currentMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::configMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::pidMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::miscMenu()
                {
                    int choice;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| {}" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                                break;

                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::start()
                {
                    std::cout << "Roboclaw hardware testing and diagnostics" << std::endl;
                    std::cout << "WARNING: Physical access to the hardware is required. Ensure that it is ";
                    std::cout << "running off battery power, and the wheels should be free spinning." << std::endl;
                    std::cout << std::endl << "These tests aim to reveal problems that unit testing can't account for" << std::endl;
                    std::cout << std::endl << std::endl << std::endl;
                    mainMenu();

                }

            }
        }
    }
}
