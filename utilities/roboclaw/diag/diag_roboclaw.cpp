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
                        std::cout << "8| PID diagnostics" << std::endl;
                        std::cout << "9| miscellaneous" << std::endl;
                        std::cout << "10| remove a roboclaw from the list" << std::endl;
                        std::cout << "11| reset list back to original (as provided in Constructor)" << std::endl;
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
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
                                clawVersion();
                                break;
                            }
                            case 2:
                            {
                                clawStatus();
                                break;
                            }
                            case 3:
                            {
                                voltageMenu();
                                break;
                            }
                            case 4:
                            {
                                currentMenu();
                                break;
                            }
                            case 5:
                            {
                                simpleDrive();
                                break;
                            }
                            case 6:
                            {
                                driveMenu();
                                break;
                            }
                            case 7:
                            {
                                encoderMenu();
                                break;
                            }
                            case 8:
                            {
                                pidMenu();
                                break;
                            }
                            case 9:
                            {
                                miscMenu();
                                break;
                            }
                            case 10:
                            {
                                std::cout << "Enter the index of the claw to remove" << std::endl;
                                std::cin.clear();
                                std::cin >> n;
                                removeClaw(n);
                                break;
                            }
                            case 11:
                            {
                                std::cout << "Resetting roboclaw list back to default" << std::endl;
                                recoverClaws();
                            }
                            default:
                            {
                                std::cout << "invalid choice" << std::endl;
                            }
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
                    bool defaultsSet = 1;

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
                            std::cout << "Disabling revert option (option 7) " << std::endl;
                            defaultsSet = 0;
                        }
                    }
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw voltage Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "Getters (caution)" << std::endl;
                        std::cout << "1| read main battery voltage" << std::endl;
                        std::cout << "2| read logic battery voltage" << std::endl;
                        std::cout << "3| read min/max main voltage" << std::endl;
                        std::cout << "4| read min/max logic voltage" << std::endl;
                        std::cout << "Setters (caution)" << std::endl;
                        std::cout << "5| set main battery voltage min/max's" << std::endl;
                        std::cout << "6| set logic battery voltage min/max's" << std::endl;
                        std::cout << "7| set voltages back to initial values" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
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
                                std::cin.clear();
                                std::cin >> c;
                                minmax[0] = c * units::V;
                                std::cout << "Enter a value in volts for the maximum main battery voltage (min 6, max 34)" << std::endl;
                                std::cin.clear();
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
                                std::cin.clear();
                                std::cin >> c;
                                minmax[0] = c * units::V;
                                std::cout << "Enter a value in volts for the maximum logic battery voltage (min 6, max 34)" << std::endl;
                                std::cin.clear();
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
                                if(defaultsSet)
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
                                }
                                else
                                {
                                    std::cout << "Defaults not available" << std::endl;
                                }
                                break;
                            }
                            default:
                                std::cout << "invalid choice" << std::endl;
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin.clear();
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
                    std::cout << std::endl << "Enter any number to continue..." << std::endl;
                    std::cin.clear();
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
                    std::cout << std::endl << "Enter any number to continue..." << std::endl;
                    std::cin.clear();
                    std::cin >> n;
                }

                void Diag::simpleDrive()
                {
                    int16_t duty;
                    std::cout << "Enter a duty (-32767 - 32767, where 32767 is max duty)" <<std::endl;
                    std::cout << "Enter 0 to stop the motor(s)" << std::endl;
                    std::cin.clear();
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
					std::array<int, 2> s1, s2, v1, v2;
					double d, acc;
                    MotorDynamics dynamics, stop;
                    units::Velocity v;
                    units::Distance dist;
                    units::Acceleration accel;
                    units::Time t;
					stop.setSpeed(0);
					stop.setDistance(0);
                    std::cout << std::endl << "Note that the drive functions will throw if the respective dynamics have not been set." << std::endl;
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Dynamics Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| drive w/ velocity" << std::endl;
                        std::cout << "2| drive w/ velocity, acceleration" << std::endl;
                        std::cout << "3| drive w/ velocity, distance " << std::endl;
                        std::cout << "4| drive w/ velocity, distance, acceleration " << std::endl;
                        std::cout << "5| drive w/ velocity, distance, acceleration, deceleration " << std::endl;
                        std::cout << "6| set velocity" << std::endl;
                        std::cout << "7| set distance" << std::endl;
                        std::cout << "8| set acceleration" << std::endl;
                        std::cout << "9| set deceleration" << std::endl;
						std::cout << "10| accel test" << std::endl;
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
                        std::cin >> choice;

                        switch(choice)
                        {
                            case 0:
                            {
                                std::cout << "Exiting" << std::endl;
                                break;
                            }
							case 3:
							case 4:
                            case 5:
							{
								for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setDynamics(dynamics, 0);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        std::cout << "Ensure that the dynamics are properly set." << std::endl;
                                    }
                                }
								for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setDynamics(stop, 0);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        std::cout << "Ensure that the dynamics are properly set." << std::endl;
                                    }
                                }
								break;
							}
                            case 1:
                            case 2:
                            {
								for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setDynamics(dynamics, 0);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        std::cout << "Ensure that the dynamics are properly set." << std::endl;
                                    }
                                }
								break;
							}
                                                   
                            case 6:
                            {
                                //velocity
                                m_velocity = velocityPrompt();
                                dynamics.setSpeed(m_velocity);
                                break;
                            }
                            case 7:
                            {
                                //dist
                                std::cout << "Distance must be positive" << std::endl;
                                m_dist = distancePrompt();
                                if (dist() < 0)
                                {
                                    std::cout << "Distance must be positive, changing sign of distance" << std::endl;
                                    m_dist *=-1;
                                }
                                dynamics.setDistance(m_dist);
                                break;
                            }
                            case 8:
							{
								
                                //acceleration
                                std::cout << "acceleration = dist / time^2" << std::endl;

                                std::cout << "Choose a distance unit" << std::endl;
                                std::cout << "1: meter, 2: ft, 3: cm , 4: in, 5: mm" << std::endl;
                                std::cin.clear();
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
                                std::cin.clear();
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
                                std::cin.clear();
                                std::cin >> d;
								
                                accel = dist / (t * t);
                                accel *= d;
                            
								std::cout << "debugging 1" << std::endl;
								m_accel = accel;
								dynamics.setAcceleration(m_accel);
                       
                                break;
							}
                            case 9:
                            {
                                //acceleration
                                std::cout << "acceleration = dist / time^2" << std::endl;

                                std::cout << "Choose a distance unit" << std::endl;
                                std::cout << "1: meter, 2: ft, 3: cm , 4: in, 5: mm" << std::endl;
                                std::cin.clear();
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
                                std::cin.clear();
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
                                std::cin.clear();
                                std::cin >> d;
								
                                accel = dist / (t * t);
                                accel *= d;
                                                              
								m_decel = accel;
								dynamics.setDeceleration(m_decel);
							
                                break;
                            }
							case 10:
							{
								//test acceleration
								int numb=0, dumb_iterator=0, j;
								std::vector<int> averagesM1, averagesM2;
								std::cout << "Enter number of iterations to do before prompting" << std::endl;
								std::cin.clear();
								std::cin >> j;
								if(dynamics.getDType() != MotorDynamics::DType::kSpeedAccel)
								{
									std::cout << " Speed/acceleration dynamic type required" << std::endl;
									break;
								}
								for(int i=0; i<m_roboclaws.size(); i++)
								{
									try
									{
										m_roboclaws[i]->resetEncoders();
										m_roboclaws[i]->setDynamics(dynamics, 0);
										averagesM1.push_back(0);
										averagesM2.push_back(0);
									}
									catch(const std::exception &e)
									{
										std::cout << "claw "<< i << "| " << e.what() << std::endl;
										std::cout << "Ensure that the dynamics are properly set." << std::endl;
									}
								}
								s1[0] = m_roboclaws[0]->readEncoderRaw(Roboclaw::Motor::kM1);
								v1[0] = m_roboclaws[0]->readEncoderVelocityRaw(Roboclaw::Motor::kM1);
									
								s1[1] = m_roboclaws[0]->readEncoderRaw(Roboclaw::Motor::kM2);
								v1[1] = m_roboclaws[0]->readEncoderVelocityRaw(Roboclaw::Motor::kM2);
							
								do
								{
									for(int i=0; i<m_roboclaws.size(); i++)
									{
										try
										{
											s2[0] = m_roboclaws[i]->readEncoderRaw(Roboclaw::Motor::kM1);
											v2[0] = m_roboclaws[i]->readEncoderVelocityRaw(Roboclaw::Motor::kM1);
											acc = (pow(v2[0], 2) - pow(v1[0], 2)) / (2.0 * (s2[0] - (s1[0]+1)));
											if((dumb_iterator % 10) == 0)
											{
												s1[0] = s2[0];
												v1[0] = v2[0];
											}
											std::cout <<"Claw " << i << " , motor M1&M2, accel(M1): ";
											std::cout << acc << std::endl;
											if(acc > 10000 || acc < -10000)
											{
												acc = 0;
											}
											averagesM1[i]+= acc;
											
											s2[1] = m_roboclaws[i]->readEncoderRaw(Roboclaw::Motor::kM2);
											v2[1] = m_roboclaws[i]->readEncoderVelocityRaw(Roboclaw::Motor::kM2);
											acc = (pow(v2[1], 2) - pow(v1[1], 2)) / (2.0 * (s2[1] - (s1[1]+1)));
											
											if((dumb_iterator % 10) == 0)
											{
												s1[1] = s2[1];
												v1[1] = v2[1];
											}	
											std::cout <<"Claw " << i << " , motor M1&M2, accel(M2): ";
											std::cout << acc << std::endl;
											if(acc > 10000 || acc < -10000)
											{
												acc = 0;
											}
											averagesM2[i]+=acc;
										}
										catch(const std::exception &e)
										{	 
											std::cout << "claw "<< i << "| " << e.what() << std::endl;
											std::cout << "Ensure that the dynamics are properly set." << std::endl;
										}
										
									}
									if(dumb_iterator > j)
									{
										std::cout << std::endl << "Enter any number to continue, -1 to stop" << std::endl;
										std::cin.clear();
										std::cin >> numb;
									}
									dumb_iterator++;
								} while(numb != -1);
								for(int i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        m_roboclaws[i]->setDynamics(stop, 0);
										averagesM1[i]/= dumb_iterator;
										averagesM2[i]/= dumb_iterator;
										std::cout <<"Claw " << i << " , motor M1&M2, avg accel(M1): ";
										std::cout << std::dec << averagesM1[i] << std::endl;
										std::cout <<"Claw " << i << " , motor M1&M2, avg accel(M2): ";
										std::cout << std::dec << averagesM2[i] << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        std::cout << "Ensure that the dynamics are properly set." << std::endl;
                                    }
                                }
								
								std::cout << "fin" << std::endl;
								break;
							}
                            default:
                            {
                                std::cout << "invalid choice" << std::endl;
                            }
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin.clear();
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
                        std::cout << "8| Motor selection (1,2, or both. default is both)" << std::endl;


                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
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
                                std::cout << "Successfully read " << i << " roboclaw's encoder values" << std::endl;
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
                                std::cout << "Successfully read " << i << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 3:
                            {
                                //set encoder raw tick count
                                std::cout << "Enter amount of encoder ticks to set to" <<std::endl;
                                std::cin.clear();
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
                                std::cout << "Successfully set " << i << " roboclaw's encoder values" << std::endl;
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
                                std::cout << "Successfully set " << i << " roboclaw's encoder values" << std::endl;
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
                                    std::cout << "Successfully read " << i << " roboclaw's encoder values" << std::endl;
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
                                    std::cout << "Successfully read " << i << " roboclaw's encoder values" << std::endl;
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
                                std::cout << "Successfully reset " << i << " roboclaw's encoder values" << std::endl;
                                break;
                            }
                            case 8:
                            {
                                std::cout << "Enter a number between 0 and 2" << std::endl;
                                std::cout << "0 -> M1" << std::endl;
                                std::cout << "1 -> M2" << std::endl;
                                std::cout << "2 -> M1&M2 (default)" << std::endl;
                                std::cin.clear();
                                std::cin >> motors;
                                if((motors < 0 )||(motors > 2))
                                {
                                    std::cout << "Invalid entry, resetting to default" << std::endl;
                                    motors = 2;
                                }
                                break;
                            }
                            default:
                                std::cout << "invalid choice" << std::endl;
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin.clear();
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::currentMenu()
                {
                    int choice, n, motors=2, i=0;
                    bool defaultsSet = 1;
                    uint32_t temp;
                    std::vector<units::Current> defaultMaxM1;
                    std::vector<units::Current> defaultMaxM2;
                    units::Current current;

                    for(i=0; i<m_roboclaws.size(); i++)
                    {
                        try
                        {
                            defaultMaxM1.push_back(m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM1));
                            defaultMaxM2.push_back(m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM2));
                        }
                        catch(const std::exception &e)
                        {
                            std::cout << "Issue reading claw " << i << " max current, " << e.what() << std::endl;
                            std::cout << "Disabling revert option (option 7) " << std::endl;
                            defaultsSet = 0;
                        }
                    }

                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Current Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| read current" << std::endl;
                        std::cout << "2| set max current" << std::endl;
                        std::cout << "3| read max current" << std::endl;
                        std::cout << "4| revert to original max current (upon entering current menu)" << std::endl;
                        std::cout << "5| Motor selection (1,2, or both. default is both)" << std::endl;
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
                        std::cin >> choice;

                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                            {
                                //read current
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1, current value: ";
                                                std::cout << m_roboclaws[i]->readCurrent(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor M2, current value: ";
                                                std::cout << m_roboclaws[i]->readCurrent(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, current value(1): ";
                                                std::cout << m_roboclaws[i]->readCurrents()[0] << std::endl;
                                                std::cout << "Current value(2) " << m_roboclaws[i]->readCurrents()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                    std::cout << "Successfully read " << i << " roboclaw's current values" << std::endl;
                                    break;
                            }
                            case 2:
                            {
                                //setMaxCurrent
                                std::cout << "Enter maximum current (amps)" << std::endl;
                                std::cin.clear();
                                std::cin >> temp;
                                current = units::A * temp;
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                m_roboclaws[i]->setMaxCurrent(Roboclaw::Motor::kM1, current);
                                                break;
                                            }
                                            case 2:
                                            {
                                                m_roboclaws[i]->setMaxCurrent(Roboclaw::Motor::kM1, current);
                                            }
                                            case 1:
                                            {
                                                m_roboclaws[i]->setMaxCurrent(Roboclaw::Motor::kM2, current);
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set " << i << " roboclaw's max current values" << std::endl;
                                break;
                            }
                            case 3:
                            {
                                //readMaxCurrent
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1, max current value: ";
                                                std::cout << m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor M2, max current value: ";
                                                std::cout << m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, max current value(1): ";
                                                std::cout << m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM2) << std::endl;
                                                std::cout << "max current value(2) " << m_roboclaws[i]->readMaxCurrent(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i << " roboclaw's max current values" << std::endl;
                                break;
                            }
                            case 4:
                            {
                                //revert current changes
                                if(defaultsSet)
                                {
                                    for(i=0; i<m_roboclaws.size(); i++)
                                    {
                                        try
                                        {
                                            m_roboclaws[i]->setMaxCurrent(Roboclaw::Motor::kM1, defaultMaxM1[i]);
                                            m_roboclaws[i]->setMaxCurrent(Roboclaw::Motor::kM2, defaultMaxM2[i]);
                                        }
                                        catch(const std::exception &e)
                                        {
                                            std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        }
                                    }
                                }
                                else
                                {
                                    std::cout << "Defaults are not available." << std::endl;
                                }
                                break;
                            }
                            case 5:
                            {

                                std::cout << "Enter a number between 0 and 2" << std::endl;
                                std::cout << "0 -> M1" << std::endl;
                                std::cout << "1 -> M2" << std::endl;
                                std::cout << "2 -> M1&M2 (default)" << std::endl;
                                std::cin.clear();
                                std::cin >> motors;
                                if((motors < 0 )||(motors > 2))
                                {
                                    std::cout << "Invalid entry, resetting to default" << std::endl;
                                    motors = 2;
                                }
                                break;
                            }
                            default:
                            {
                                std::cout << "invalid choice" << std::endl;
                            }
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin.clear();
                        std::cin >> n;
                    } while(choice != 0);
                }

                void Diag::pidMenu()
                {
                    int choice, n, i=0, motors=2, j=0;
					float pid=0;
					uint32_t qpps;
					VelocityPIDParameters vparams;
					PositionPIDParameters pparams;
					
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw {} Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| read velocity PID" << std::endl;
                        std::cout << "2| read position PID" << std::endl;
                        std::cout << "3| set velocity PID" << std::endl;
						std::cout << "4| set position PID" << std::endl;
						std::cout << "5| set velocity PID parameters" << std::endl;
						std::cout << "6| set position PID parameters" << std::endl;
						std::cout << "7| Motor selection (1,2, or both. default is both)" << std::endl;
			
		
                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
                        std::cin >> choice;
                        switch(choice)
                        {
                            case 0:
                                std::cout << "Exiting" << std::endl;
                                break;
                            case 1:
                            {
								//read velocity PID
								for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {
											case 2:
                                            case 0:
                                            {
												vparams = m_roboclaws[i]->readVelocityPID(Roboclaw::Motor::kM1);
                                                std::cout <<"Claw " << i << " , motor M1, velocity, P: " << vparams.kp << "   I: ";
												std::cout << vparams.ki << "   D: " << vparams.kd << "   QPPS: " << vparams.qpps;
                                                if(motors != 2)
												{
													break;
												}
                                            }
                                            case 1:
                                            {
												vparams = m_roboclaws[i]->readVelocityPID(Roboclaw::Motor::kM2);
                                                std::cout <<"Claw " << i << " , motor M1, velocity, P: " << vparams.kp << "   I: ";
												std::cout << vparams.ki << "   D: " << vparams.kd << "   QPPS: " << vparams.qpps << std::endl;
                                                
												break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i << " roboclaw's velocity PID values" << std::endl;
								break;
							}
							case 2:
							{
								//read position PID
								for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {
											case 2:
                                            case 0:
                                            {
												pparams = m_roboclaws[i]->readPositionPID(Roboclaw::Motor::kM1);
                                                std::cout <<"Claw " << i << " , motor M1, position, P: " << pparams.kp << "   I: ";
												std::cout << pparams.ki << "   D: " << pparams.kd << std::endl;
												std::cout << "kiMax: " << pparams.kiMax << "    deadzone: " << pparams.deadzone << "    min: ";
												std::cout << pparams.min << "    max: " << pparams.max << std::endl;
                                                if(motors != 2)
												{
													break;
												}
                                            }
                                            case 1:
                                            {
												pparams = m_roboclaws[i]->readPositionPID(Roboclaw::Motor::kM2);
                                                std::cout <<"Claw " << i << " , motor M2, position, P: " << pparams.kp << "   I: ";
												std::cout << pparams.ki << "   D: " << pparams.kd << std::endl;
												std::cout << "kiMax: " << pparams.kiMax << "    deadzone: " << pparams.deadzone << "    min: ";
												std::cout << pparams.min << "    max: " << pparams.max << std::endl;
												break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i << " roboclaw's position PID values" << std::endl;
								break;
							}
							case 3:
							{
								//set velocity PID
								for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {
											case 2:
                                            case 0:
                                            {
												m_roboclaws[i]->setVelocityPID(Roboclaw::Motor::kM1, vparams);
                                                if(motors != 2)
												{
													break;
												}
                                            }
                                            case 1:
                                            {
												m_roboclaws[i]->setVelocityPID(Roboclaw::Motor::kM2, vparams);
												break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set " << i << " roboclaw's velocity PID values" << std::endl;
								break;
							}
							case 4:
							{
								//set position PID
								for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {
											case 2:
                                            case 0:
                                            {
												m_roboclaws[i]->setPositionPID(Roboclaw::Motor::kM1, pparams);
                                                if(motors != 2)
												{
													break;
												}
                                            }
                                            case 1:
                                            {
												m_roboclaws[i]->setPositionPID(Roboclaw::Motor::kM2, pparams);
												break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set " << i << " roboclaw's velocity PID values" << std::endl;
								break;
							}
							case 5:
							{
								std::cout << "Enter a number for proportional gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								vparams.kp = pid;
								std::cout << "Enter a number for integral gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								vparams.ki = pid;
								std::cout << "Enter a number for derivative gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								vparams.kd = pid;
								std::cout << "Enter a number for quadrature pulses per second" << std::endl;
								std::cin.clear();
								std::cin >> qpps;
								vparams.qpps = qpps;
								
								break;
							}
							case 6:
							{
								std::cout << "Enter a number for proportional gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								pparams.kp = pid;
								std::cout << "Enter a number for integral gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								pparams.ki = pid;
								std::cout << "Enter a number for derivative gain" << std::endl;
								std::cin.clear();
								std::cin >> pid;
								pparams.kd = pid;
								std::cout << "Enter a number for kiMax" << std::endl;
								std::cin.clear();
								std::cin >> qpps;
								pparams.kiMax = qpps;
								std::cout << "Enter a number for deadzone" << std::endl;
								std::cin.clear();
								std::cin >> qpps;
								pparams.deadzone = qpps;
								std::cout << "Enter a number for min" << std::endl;
								std::cin.clear();
								std::cin >> qpps;
								pparams.min = qpps;
								std::cout << "Enter a number for max" << std::endl;
								std::cin.clear();
								std::cin >> qpps;
								pparams.max = qpps;
								break;
							}
							case 7:
							{

								std::cout << "Enter a number between 0 and 2" << std::endl;
								std::cout << "0 -> M1" << std::endl;
								std::cout << "1 -> M2" << std::endl;
								std::cout << "2 -> M1&M2 (default)" << std::endl;
								std::cin.clear();
								std::cin >> motors;
								if((motors < 0 )||(motors > 2))
								{
									std::cout << "Invalid entry, resetting to default" << std::endl;
									motors = 2;
								}
								break;
							}
							default:
							{
								std::cout << "invalid choice" << std::endl;
							}
                        }
                    } while(choice != 0);
                }

                void Diag::miscMenu()
                {
                    int choice, n, i=0, motors=2;
                    Config cfg;
                    uint16_t encodedCfg;
                    bool cfgSet=0;
                    std::vector<Config> defaultCfg;
                    do {
                        std::cout << std::endl << std::endl << "Roboclaw Miscellaneous Menu" << std::endl;
                        std::cout << "0| exit" << std::endl;
                        std::cout << "1| read temperature" << std::endl;
                        std::cout << "2| read buffer length" << std::endl;
                        std::cout << "3| read config" << std::endl;
                        std::cout << "Warning: setting config can break things" << std::endl;
                        std::cout << "4| set config via encoded hex" << std::endl;
                        std::cout << "Motor selection applies to read buffer length" << std::endl;
                        std::cout << "5| Motor selection (1,2, or both. default is both) " << std::endl;
                        std::cout << "6| set config back to state after first read" << std::endl;

                        std::cout << std::endl << std::endl << "Enter a choice" << std::endl;
                        std::cin.clear();
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
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        std::cout << "Temperature of claw " << i << ": "<< m_roboclaws[i]->readTemperature() << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }

                                }
                                std::cout << "Successfully read the temperature of " << i << " roboclaws" << std::endl;
                                break;
                            }
                            case 2:
                            {
                                //read buffer Length
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        switch(motors)
                                        {

                                            case 0:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1, buffer length: ";
                                                std::cout << m_roboclaws[i]->readBufferLen(Roboclaw::Motor::kM1) << std::endl;
                                                break;
                                            }
                                            case 1:
                                            {
                                                std::cout <<"Claw " << i << " , motor M2, buffer length: ";
                                                std::cout << m_roboclaws[i]->readBufferLen(Roboclaw::Motor::kM2) << std::endl;
                                                break;
                                            }
                                            case 2:
                                            {
                                                std::cout <<"Claw " << i << " , motor M1&M2, buffer length(M1): ";
                                                std::cout << m_roboclaws[i]->readBufferLens()[0] << std::endl;
                                                std::cout << "buffer length m2: " << m_roboclaws[i]->readBufferLens()[1] << std::endl;
                                                break;
                                            }
                                        }
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully read " << i << " roboclaw's buffer lengths" << std::endl;
                                break;
                            }
                            case 3:
                            {
                                //read config1
                                std::cout << "See Roboclaw manual if you want to decode the output." << std::endl;
                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    try
                                    {
                                        cfg = m_roboclaws[i]->getConfig();
                                        if(!cfgSet)
                                        {
                                            defaultCfg.push_back(cfg);
                                        }
                                        std::cout << "claw " << i <<": " <<  std::hex << cfg.get() << std::endl;
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                if(defaultCfg.size() >= m_roboclaws.size())
                                {
                                    cfgSet = true;
                                }
                                std::cout << "Successfully read config of " << i << "roboclaws" << std::endl;
                                break;
                            }
                            case 4:
                            {
                                //set config
                                std::cout << "Changes to config can break communications, or mess with voltages" << std::endl;

                                for(i=0; i<m_roboclaws.size(); i++)
                                {
                                    std::cout << "Enter hex of encoded config settings to set for claw " << i << std::endl;
                                    std::cin.clear();
                                    std::cin >> std::hex >> encodedCfg;
                                    cfg.set(encodedCfg);
                                    try
                                    {
                                        m_roboclaws[i]->setConfig(cfg);
                                    }
                                    catch(const std::exception &e)
                                    {
                                        std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                    }
                                }
                                std::cout << "Successfully set config of " << i << "roboclaws" << std::endl;
                                break;
                            }
                            case 5:
                            {
                                std::cout << "Enter a number between 0 and 2" << std::endl;
                                std::cout << "0 -> M1" << std::endl;
                                std::cout << "1 -> M2" << std::endl;
                                std::cout << "2 -> M1&M2 (default)" << std::endl;
                                std::cin.clear();
                                std::cin >> motors;
                                if((motors < 0 )||(motors > 2))
                                {
                                    std::cout << "Invalid entry, resetting to default" << std::endl;
                                    motors = 2;
                                }
                                break;
                            }
                            case 6:
                            {
                                if(cfgSet)
                                {
                                    std::cout << "Reverting changes to settings" << std::endl;
                                    for(i=0; i<m_roboclaws.size(); i++)
                                    {
                                        try
                                        {
                                            m_roboclaws[i]->setConfig(defaultCfg[i]);
                                        }
                                        catch(const std::exception &e)
                                        {
                                            std::cout << "claw "<< i << "| " << e.what() << std::endl;
                                        }
                                    }
                                    std::cout << "Successfully set config of " << i << "roboclaws" << std::endl;
                                }
                                else
                                {
                                    std::cout << "Default configs are not available" << std::endl;
                                }
                                break;
                            }
                            default:
                            {
                                std::cout << "invalid choice" << std::endl;
                                break;
                            }
                        }
                        std::cout << std::endl << "Enter any number to continue..." << std::endl;
                        std::cin.clear();
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
                    int n;
                    double mag;
                    units::Distance dist;
                    std::cout << "Choose a distance unit" << std::endl;
                    std::cout << "1: meter, 2: ft, 3: cm , 4: in, 5: mm" << std::endl;
                    std::cin.clear();
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
                    std::cin.clear();
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
                    int n;
                    double mag;
                    std::cout << "Velocity: distance / time: " << std::endl;
                    dist = distancePrompt();

                    std::cout << "Select a time unit" << std::endl;
                    std::cout << "1: seconds, 2: ms, 3: minute, 4: hour" << std::endl;
                    std::cin.clear();
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
                    std::cin.clear();
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
