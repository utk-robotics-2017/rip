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
                    m_roboclaws_original = roboclaws;
                }
                //Todo: motor select, add motor back
                void Diag::mainMenu()
                {
                    int choice, n;

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Diag Main Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| readVersion" << std::endl;
                        std::cout << "2| readStatus" << std::endl;
                        std::cout << "3| voltage diagnostics" << std::endl;
                        std::cout << "4| current diagnostics" << std::endl;
                        std::cout << "5| basic (duty) driving diagnostics" << std::endl;
                        std::cout << "6| advanced driving/dynamics diagnostics" << std::endl;
                        std::cout << "7| encoder diagnostics" << std::endl;
                        std::cout << "8| configuration diagnostics" << std::endl;
                        std::cout << "9| PID diagnostics" << std::endl;
                        std::cout << "10| miscellaneous" << std::endl;
                        std::cout << "11| remove a roboclaw from the list" << std::endl;
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
                                simpleDrive();
                                break;
                            case 6:
                                driveMenu();
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
                            case 11:
                            {
                                std::cout << "Enter the index of the claw to remove" << std::endl;
                                std::cin >> n;
                                removeClaw(n);
                                break;
                            }
                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                    } while(choice != 0);
                }

                void Diag::voltageMenu()
                {
                    int choice, n;
                    uint32_t c;
                    std::vector<std::array<units::Voltage, 2>> defaultMain;
                    std::vector<std::array<units::Voltage, 2>> defaultLogic;
                    std::array<units::Voltage, 2> minmax;
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
                                break;
                            }
                            case 2:
                            {
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        v=m_roboclaws[i]->readLogicBatteryVoltage();
                                        std::cout << "Claw " << i << " Logic battery voltage: " << v() << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            case 3:
                            {
                                //min max main
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        minmax = m_roboclaws[i]->readMinMaxMainVoltages();
                                        std::cout << "Claw " << i << " Main battery min voltage: " << minmax[0].to(units::V) << std::endl;
                                        std::cout << "Claw " << i << " Main battery max voltage: " << minmax[1].to(units::V) << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            case 4:
                            {
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        minmax = m_roboclaws[i]->readMinMaxLogicVoltages();
                                        std::cout << "Claw " << i << " logic battery min voltage: " << minmax[0].to(units::V) << std::endl;
                                        std::cout << "Claw " << i << " logic battery max voltage: " << minmax[1].to(units::V) << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            case 5:
                            {
                                //set min max main
                                std::cout << "Enter a value in volts for the minimum main battery voltage (min 6, max 34)" << std::endl;
                                std::cin >> c;
                                minmax[0] = c * units::V;
                                std::cout << "Enter a value in volts for the maximum main battery voltage (min 6, max 34)" << std::endl;
                                std::cin >> c;
                                minmax[1] = c * units::V;
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setMainVoltages(minmax[0], minmax[1]);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            case 6:
                            {
                                std::cout << "Enter a value in volts for the minimum logic battery voltage (min 6, max 34)" << std::endl;
                                std::cin >> c;
                                minmax[0] = c * units::V;
                                std::cout << "Enter a value in volts for the maximum logic battery voltage (min 6, max 34)" << std::endl;
                                std::cin >> c;
                                minmax[1] = c * units::V;

                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setLogicVoltages(minmax[0], minmax[1]);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            case 7:
                            {
                                std::cout << "setting roboclaw's back to their prior values (upon entering voltage menu)" << std::endl;
                                for(int i=0;i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setMainVoltages(defaultMain[i][0], defaultMain[i][1]);
                                        m_roboclaws[i]->setLogicVoltages(defaultLogic[i][0], defaultLogic[i][1]);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw " << i << "| " << e.what() << std::endl;
                                    }
                                }
                                break;
                            }
                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                        std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                        std::cin >> n;
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

                void Diag::simpleDrive()
                {
                    int16_t duty;
                    std::cout << "Enter a duty (-32767 - 32767, where 32767 is max duty)" <<std::endl;
                    std::cout << "Enter 0 to stop the motor(s)" << std::endl;
                    std::cin >> duty;

                    for(int i=0; i<m_roboclaws.size(); i++)
                    {
                        try
                        {
                            m_roboclaws[i]->drive(duty);
                        }
                        catch(const std::exception &e)
                        {
                            std::cout << "claw "<< i << "| " << e.what() << std::endl;
                        }
                    }
                }

                void Diag::driveMenu()
                {
                    int choice, n, mag=0;
                    uint32_t mag2=0;
                    MotorDynamics dynamics;
                    units::Velocity v;
                    units::Distance dist;
                    units::Acceleration accel;
                    units::Time t;

                    std::cout << std::endl << "Note that the drive functions will throw if the respective dynamics have not been set." << std::endl;
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
                            {
                                std::cout << "Exiting" << std::endl;
                                break;
                            }
                            case 1:
                            case 2:
                            case 3:
                            case 4:
                            case 5:
                            case 6:
                            {
                                for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setDynamics(dynamics);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        std::cout << "Ensure that the dynamics are properly set." << std::endl;
                                    }
                                }
                            }
                            case 7:
                            {
                                //velocity
                                v = velocityPrompt();
                                dynamics.setSpeed(v);
                                break;
                            }
                            case 8:
                            {
                                //dist
                                std::cout << "Distance must be positive" << std::endl;
                                dist = distancePrompt();
                                if (dist() < 0)
                                {
                                    std::cout << "Distance must be positive, changing sign of distance" << std::endl;
                                    dist *=-1;
                                }
                                dynamics.setDistance(dist);
                                break;
                            }
                            case 9:
                            case 10:
                            {
                                //acceleration
                                std::cout << "acceleration = dist / time^2" << std::endl;

                                std::cout << "Choose a distance unit" << std::endl;
                                std::cout << "1: meter, 2: ft, 3: cm , 4: in, 5: mm" << std::endl;
                                std::cin >> n;
                                switch(n)
                                {
                                    case 1:
                                    {
                                        dist = units::m;
                                        break;
                                    }
                                    case 2:
                                    {
                                        dist = units::ft;
                                        break;
                                    }
                                    case 3:
                                    {
                                        dist = units::cm;
                                        break;
                                    }
                                    case 4:
                                    {
                                        dist = units::in;
                                        break;
                                    }
                                    case 5:
                                    {
                                        dist = units::mm;
                                        break;
                                    }
                                    default:
                                    {
                                        std::cout << "Enter a valid selection, defaulting to feet" << std::endl;
                                        dist = units::ft;
                                    }
                                }

                                std::cout << "Select a time unit" << std::endl;
                                std::cout << "1: seconds, 2: ms, 3: minute, 4: hour" << std::endl;
                                std::cin >> n;
                                switch(n)
                                {
                                    case 1:
                                    {
                                        t = units::s;
                                        break;
                                    }
                                    case 2:
                                    {
                                        t = units::ms;
                                        break;
                                    }
                                    case 3:
                                    {
                                        t = units::minute;
                                        break;
                                    }
                                    case 4:
                                    {
                                        t = units::hr;
                                        break;
                                    }
                                    default:
                                    {
                                        std::cout << "Enter a valid selection, defaulting to seconds" << std::endl;
                                        t = units::s;
                                    }
                                }

                                std::cout << "Enter a positive magnitude for acceleration" << std::endl;
                                std::cin >> mag2;
                                accel = dist / (t * t);
                                accel *= mag2;
                                if(n == 9)
                                {
                                    dynamics.setAcceleration(accel);
                                }
                                else
                                {
                                    dynamics.setDeceleration(accel);
                                }
                                break;
                            }
                            default:
                            {
                                std::cout << "invalid choice, git good" << std::endl;
                            }
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::encoderMenu()
                {
                    int choice, n, motors=2, i=0, ticks=0;
                    units::Distance dist;
                    units::Velocity velocity;
                    /*
                    Motors:
                    0 -> m1
                    1 -> m2
                    2 -> m1&m2 -> default
                    */
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Encoder Menu" << std::endl;

                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| read encoder raw tick count" << std::endl;
                        std::cout << "2| read encoder (distance)" << std::endl;
                        std::cout << "3| set encoder raw tick count" << std::endl;
                        std::cout << "4| set encoder via distance" << std::endl;
                        std::cout << "5| read encoder velocity (raw)" << std::endl;
                        std::cout << "6| read encoder velocity" << std::endl;
                        std::cout << "7| reset encoders to 0" << std::endl;
                        std::cout << "8| Motor selection (1,2, or both. defaults is both)" << std::endl;


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
                                //raw encoder tick count
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor kM1, raw encoder value: ";
                                                std::cout << m_roboclaws[i]->readEncoderRaw(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor kM2, raw encoder value: ";
                                                std::cout << m_roboclaws[i]->readEncoderRaw(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, raw encoder value(1): ";
                                                std::cout << m_roboclaws[i]->readEncodersRaw()[0] << std::endl;
                                                std::cout << "Raw encoder value(2) " << m_roboclaws[i]->readEncodersRaw()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i+1 << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 2:
                            {
                                //read encoder (distance)
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor kM1, encoder value: ";
                                                std::cout << m_roboclaws[i]->readEncoder(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor kM2, encoder value: ";
                                                std::cout << m_roboclaws[i]->readEncoder(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, encoder value(1): ";
                                                std::cout << m_roboclaws[i]->readEncodersRaw()[0] << std::endl;
                                                std::cout << "Encoder value(2) " << m_roboclaws[i]->readEncodersRaw()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i+1 << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 3:
                            {
                                //set encoder raw tick count
                                std::cout << "Enter amount of encoder ticks to set to" <<std::endl;
                                std::cin >> ticks;
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                m_roboclaws[i]->setEncoderRaw(Roboclaw::Motor::kM1, ticks);
                                                break;
                                            }
                                            case 2:
                                            {
                                                m_roboclaws[i]->setEncoderRaw(Roboclaw::Motor::kM1, ticks);
                                            }
                                            case 1:
                                            {
                                                m_roboclaws[i]->setEncoderRaw(Roboclaw::Motor::kM2, ticks);
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set " << i+1 << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 4:
                            {
                                //set encoder via distance
                                dist = distancePrompt();
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                m_roboclaws[i]->setEncoder(Roboclaw::Motor::kM1, dist);
                                                break;
                                            }
                                            case 2:
                                            {
                                                m_roboclaws[i]->setEncoder(Roboclaw::Motor::kM1, dist);
                                            }
                                            case 1:
                                            {
                                                m_roboclaws[i]->setEncoder(Roboclaw::Motor::kM2, dist);
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set " << i+1 << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 5:
                            {
                                //read encoder velocity (raw)
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1, raw encoder velocity value: ";
                                                std::cout << m_roboclaws[i]->readEncoderVelocityRaw(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor M2, raw encoder velocity value: ";
                                                std::cout << m_roboclaws[i]->readEncoderVelocityRaw(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, raw encoder velocity value(1): ";
                                                std::cout << m_roboclaws[i]->readEncodersVelocityRaw()[0] << std::endl;
                                                std::cout << "raw Encoder velocity value(2) " << m_roboclaws[i]->readEncodersVelocityRaw()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                    std::cout << "Successfully read " << i+1 << " roboclaw's encoder values" << std::endl;
                                }
                                break;
                            }
                            case 6:
                            {
                                //read encoder velocity
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1, encoder velocity value: ";
                                                std::cout << m_roboclaws[i]->readEncoderVelocity(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor M2, encoder velocity value: ";
                                                std::cout << m_roboclaws[i]->readEncoderVelocity(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, encoder velocity value(1): ";
                                                std::cout << m_roboclaws[i]->readEncodersVelocity()[0] << std::endl;
                                                std::cout << "Encoder velocity value(2) " << m_roboclaws[i]->readEncodersVelocityRaw()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                    std::cout << "Successfully read " << i+1 << " roboclaw's encoder values" << std::endl;
                                }
                                break;
                            }
                            case 7:
                            {
                                //reset encoders
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->resetEncoders();
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully reset " << i+1 << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 8:
                            {
                                std::cout << "Enter a number between 0 and 2" << std::endl;
                                std::cout << "0 -> M1" << std::endl;
                                std::cout << "1 -> M2" << std::endl;
                                std::cout << "2 -> M1&M2 (default)" << std::endl;
                                std::cin >> motors;
                                if((motors < 0 )||(motors > 2))
                                {
                                    std::cout << "Invalid entry, resetting to default" << std::endl;
                                    motors = 2;
                                }
                                break;
                            }
                            default:
                                std::cout << "invalid choice, git good" << std::endl;
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::currentMenu()
                {
                    int choice, n;

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
                        std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::configMenu()
                {
                    int choice, n;

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
                        std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::pidMenu()
                {
                    int choice, n;

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
                    int choice, n;

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
                        std::cout << std::endl << "Enter any number to coninue..." << std::endl;
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::removeClaw(int index)
                {
                    try
                    {
                        m_roboclaws.erase(m_roboclaws.begin() + index);
                    }
                    catch(...)
                    {
                        std::cout << "Enter a valid index." <<std::endl;
                    }
                }

                void Diag::recoverClaws()
                {
                    m_roboclaws = m_roboclaws_original;
                }

                units::Distance Diag::distancePrompt()
                {
                    //distance
                    int n, mag;
                    units::Distance dist;
                    std::cout << "Choose a distance unit" << std::endl;
                    std::cout << "1: meter, 2: ft, 3: cm , 4: in, 5: mm" << std::endl;
                    std::cin >> n;
                    switch(n)
                    {
                        case 1:
                        {
                            dist = units::m;
                            break;
                        }
                        case 2:
                        {
                            dist = units::ft;
                            break;
                        }
                        case 3:
                        {
                            dist = units::cm;
                            break;
                        }
                        case 4:
                        {
                            dist = units::in;
                            break;
                        }
                        case 5:
                        {
                            dist = units::mm;
                            break;
                        }
                        default:
                        {
                            std::cout << "Enter a valid selection, defaulting to feet" << std::endl;
                            dist = units::ft;
                        }
                    }

                    std::cout << "Enter a positive magnitude for distance" << std::endl;
                    std::cin >> mag;
                    dist *= mag;
                    return dist;
                }

                units::Velocity Diag::velocityPrompt()
                {
                    //velocity
                    units::Distance dist;
                    units::Velocity v;
                    units::Time t;
                    int n, mag;
                    std::cout << "Velocity: distance / time: " << std::endl;
                    dist = distancePrompt();

                    std::cout << "Select a time unit" << std::endl;
                    std::cout << "1: seconds, 2: ms, 3: minute, 4: hour" << std::endl;
                    std::cin >> n;
                    switch(n)
                    {
                        case 1:
                        {
                            t = units::s;
                            break;
                        }
                        case 2:
                        {
                            t = units::ms;
                            break;
                        }
                        case 3:
                        {
                            t = units::minute;
                            break;
                        }
                        case 4:
                        {
                            t = units::hr;
                            break;
                        }
                        default:
                        {
                            std::cout << "Enter a valid selection, defaulting to seconds" << std::endl;
                            t = units::s;
                        }
                    }

                    std::cout << "Enter a signed magnitute" <<std::endl;
                    std::cin >> mag;
                    v = dist / t;
                    v*= mag;
                    return v;

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
