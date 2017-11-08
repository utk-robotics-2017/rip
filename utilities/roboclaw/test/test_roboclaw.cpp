#include "mock_roboclaw.hpp"
#include <exceptions.hpp>

#include <memory>

#include <gtest/gtest.h>

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            namespace tests
            {
                nlohmann::json j;
                using Roboclaw = mocks::MockRoboclaw;
                TEST(RoboclawCore, loadJson)
                {
                    //test some types of variables that are not json obj

                    ASSERT_THROW(std::make_shared<Roboclaw>("strayTaco"), BadJson);
                    ASSERT_THROW(std::make_shared<Roboclaw>(42), BadJson);

                    //test if json null
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    units::Time t = 3.0;
                    units::Distance d = .1;
                    j["address"] = 0x80; //uint8_t
                    j["timeout"] = t.to(units::ms);
                    j["ticks_per_rev"] = 100.0;//double
                    //test for missing entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    j["wheel_radius"] = "lol";
                    //tests for invalid values of entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    j["wheel_radius"] = d.to(units::cm);
                    j["address"] = 267;//255 is largest possible value
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), OutOfRange);
                    j["address"] = 0x80;
                    j["timeout"] = -3.0;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), OutOfRange);
                    j["timeout"] = 10.0;
                    j["ticks_per_rev"] = j;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    //test extra entries
                    j["extra"] = 1;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    j.erase("extra");
                    //wheel radius cannot = 0
                    j["wheel_radius"] = 0;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), OutOfRange);

                    j["timeout"] = t.to(units::ms);
                    j["ticks_per_rev"] = 100.0;
                    j["wheel_radius"] = d.to(units::cm);
                    //given valid parameters, does not throw an exception
                    ASSERT_NO_THROW(std::make_shared<Roboclaw>(j));
                    //TODO: Reasonableness checks/nonfatal exceptions
                }


                TEST(RoboclawCore, DriveM1Forward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    //assuming valid roboclaw object at this point due to prior test
                    std::vector<uint8_t> response={0xFF};
                    testClaw->setcResponse(response);
                    //CRC check failure throws command failure
                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes();
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //final test: given proper conditions, does not throw an exception
                    //correct size of byte array
                    testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward);
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward));
                }

                TEST(RoboclawCore, DriveM1Backward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes();
                    testClaw->printResponse();
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward));
                }

                TEST(RoboclawCore, DriveM2Forward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<int8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes();
                    testClaw->printResponse();
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],33);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //correct size of byte array
                    //final test: given proper conditions, does not throw an exception
                    //ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedForward));
                }

                TEST(RoboclawCore, DriveM2Backward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<int8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes();
                    testClaw->printResponse();
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],33);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    //final test: given proper conditions, does not throw an exception
                    //ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedBackward));
                }
                TEST(RoboclawCore, DriveForward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<int8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes();
                    testClaw->printResponse();
                    //sends correct message: addr, 34, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    ASSERT_EQ(testClaw->getLastCmd()[4],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0xff);
                    //correct size of byte array
                    //final test: given proper conditions, does not throw an exception
                    //ASSERT_NO_THROW(testClaw->drive(Roboclaw::kFullSpeedForward));
                }
                TEST(RoboclawCore, DriveBackward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<int8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes();
                    testClaw->printResponse();
                    //sends correct message: addr, 34, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    ASSERT_EQ(testClaw->getLastCmd()[4],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0x1);
                    //correct size of byte array
                    //final test: given proper conditions, does not throw an exception
                    //ASSERT_NO_THROW(testClaw->drive(Roboclaw::kFullSpeedBackward));
                }
                TEST(RoboclawCore, BatteryTests)
                {
                    FAIL() << "Complete test";
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response={8,24, 0xFF};
                    units::Voltage v;
                    std::array<units::Voltage, 2> rv;
                    //voltage out of range
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(-3 * units::V,8 * units::V), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(6, 300), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(-3,8), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(6, 300), OutOfRange);
                    //min voltage should be less than max voltage

                    //set voltage, check to see if that is actual voltage
                    testClaw->setLogicVoltages(8*units::V, 24*units::V);
                    testClaw->setcResponse(response);
                    v = testClaw->readLogicBatteryVoltage();
                    EXPECT_DOUBLE_EQ(v(), 8.0);
                    testClaw->setMainVoltages(8, 24);
                    rv=testClaw->readMinMaxMainVoltages();
                    EXPECT_DOUBLE_EQ(rv[0](), 8.0);
                    EXPECT_DOUBLE_EQ(rv[1](), 24.0);

                    //readCurrent returns diff. than actual
                    testClaw->setMaxCurrent(Roboclaw::Motor::kM1, 5*units::A);
                    ASSERT_LE(testClaw->readCurrent(Roboclaw::Motor::kM1), 5*units::A);
                    ASSERT_EQ(testClaw->readMaxCurrent(Roboclaw::Motor::kM1),5*units::A);

                }
                TEST(RoboclawCore, StatusTests)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;


                    //readstatus tests

                    FAIL() << "TODO: Implement Test";
                }
                TEST(RoboclawCore, Encoders)
                {
                    FAIL() << "Complete test";
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d;
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    //encoder velocity = 0 when not moving
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), 0.0);
                    EXPECT_DOUBLE_EQ(v(), 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0.0);
                    //do for raw as well
                    testClaw->setEncoder(Roboclaw::Motor::kM1, 1.0);
                    EXPECT_DOUBLE_EQ(d(), 1.0);
                    testClaw->resetEncoders();
                }
            }
        }
    }
}
