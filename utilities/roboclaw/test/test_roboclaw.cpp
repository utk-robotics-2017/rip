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
                    //ticks per rev should be a pos. #
                    j["ticks_per_rev"] = -100.0;
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
                    testClaw->setBytes(0);
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //final test: given proper conditions, does not throw an exception
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6);
                    testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward);
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward));
                }

                TEST(RoboclawCore, DriveM1Backward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes(0);

                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6);
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward));
                }

                TEST(RoboclawCore, DriveM2Forward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes(0);

                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],33);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6);
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedForward));
                }

                TEST(RoboclawCore, DriveM2Backward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes(0);

                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6);
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::Motor::kM2, Roboclaw::kFullSpeedBackward));
                }
                TEST(RoboclawCore, DriveForward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes(0);

                    //sends correct message: addr, 34, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    ASSERT_EQ(testClaw->getLastCmd()[4],0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0xff);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 8);
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::kFullSpeedForward));
                }
                TEST(RoboclawCore, DriveBackward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;

                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::kFullSpeedBackward), CommandFailure);
                    testClaw->setBytes(0);

                    //sends correct message: addr, 34, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    ASSERT_EQ(testClaw->getLastCmd()[4],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 8);
                    //final test: given proper conditions, does not throw an exception
                    ASSERT_NO_THROW(testClaw->drive(Roboclaw::kFullSpeedBackward));
                }
                TEST(RoboclawCore, Battery)
                {
                    /*
                    Note: Roboclaw treats numbers for voltage in deciVolts, since protocol
                    does not use floats. Current responses are increments of 10 mA.
                    */
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response={0,80};
                    units::Voltage v;
                    units::Current a;
                    std::array<units::Voltage, 2> rv;
                    testClaw->setBytes(0);
                    //voltage out of range
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(-3 * units::V,8 * units::V), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(6, 300), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(-3,8), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(6, 300), OutOfRange);
                    //min voltage should be less than max voltage
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(18, 6), OutOfRange);

                    //set voltage, check to see if that is actual voltage
                    //verify proper command transmission
                    testClaw->setLogicVoltages(8*units::V, 24*units::V);
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],58);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[2] << 8) + testClaw->getLastCmd()[3]), 80.0);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 240.0);
                    testClaw->setcResponse(response);
                    //initial voltage should be set to minimum voltage
                    v = testClaw->readLogicBatteryVoltage();
                    EXPECT_DOUBLE_EQ(v(), 8.0);
                    //verify proper command transmission
                    testClaw->setMainVoltages(8*units::V, 24*units::V);
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],57);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[2] << 8) + testClaw->getLastCmd()[3]), 80.0);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 240.0);

                    response={0,80,0,240};
                    testClaw->setcResponse(response);
                    rv=testClaw->readMinMaxMainVoltages();
                    EXPECT_DOUBLE_EQ(rv[0](), 8.0);
                    EXPECT_DOUBLE_EQ(rv[1](), 24.0);
                    v=testClaw->readMainBatteryVoltage();

                    //readCurrent returns diff. than actual
                    //verify proper command transmission
                    /*
                    Send: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0, CRC(2 bytes)]
                    */
                    testClaw->setMaxCurrent(Roboclaw::Motor::kM1, 5*units::A);
                    ASSERT_EQ(testClaw->getLastCmd()[0],0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1],133);
                    ASSERT_DOUBLE_EQ(static_cast<uint32_t>((testClaw->getLastCmd()[2] << 8*3) + (testClaw->getLastCmd()[3] << 8*2) + (testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 500.0);
                    //min current should always be 0
                    ASSERT_DOUBLE_EQ(static_cast<uint32_t>((testClaw->getLastCmd()[6] << 8*3) + (testClaw->getLastCmd()[7] << 8*2) + (testClaw->getLastCmd()[8] << 8) + testClaw->getLastCmd()[9]), 0.0);
                    //Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
                    response={0,0,1, 0xF4};
                    testClaw->setcResponse(response);
                    ASSERT_LE(testClaw->readCurrent(Roboclaw::Motor::kM1), 5*units::A);
                    ASSERT_EQ(testClaw->readMaxCurrent(Roboclaw::Motor::kM1),5*units::A);
                    ASSERT_EQ(testClaw->readCurrent(Roboclaw::Motor::kM1),5*units::A);
                    ASSERT_EQ(testClaw->readCurrents()[0],5*units::A);

                }
                TEST(RoboclawCore, Status)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    std::array<bool, 17> status;
                    //readstatus tests, status normal
                    response = {0,0};
                    testClaw->setcResponse(response);
                    status = testClaw->readStatus();
                    EXPECT_TRUE(testClaw->readStatus(Roboclaw::Status::kNormal));
                    for(int i=1; i<17; i++)
                    {
                        EXPECT_FALSE(status[i]);
                    }
                    //opposite of above case, normal is false but all other statuses true
                    response = {0xFF, 0xFF};
                    testClaw->setcResponse(response);
                    status = testClaw->readStatus();
                    EXPECT_FALSE(testClaw->readStatus(Roboclaw::Status::kNormal));
                    for(int i=1; i<17; i++)
                    {
                        EXPECT_TRUE(status[i]);
                    }
                    //m1 and m2 over current warning
                    response = {0, 3};
                    testClaw->setcResponse(response);
                    EXPECT_FALSE(testClaw->readStatus(Roboclaw::Status::kNormal));
                    EXPECT_TRUE(testClaw->readStatus(Roboclaw::Status::kM1OverCurrentWarning));
                    EXPECT_TRUE(testClaw->readStatus(Roboclaw::Status::kM2OverCurrentWarning));
                }
                TEST(RoboclawCore, Encoder)
                {
                    //TODO: split into multiple test cases
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d, d2;
                    double ticks_per_rev = 360.0, wheel_radius=.04;//base unit meter?
                    uint32_t ticks, ticksc;

                    testClaw->setBytes(0);
                    //Receive: [Enc1(4 bytes), Status, CRC(2 bytes)]

                    //distance tests
                      //motor 1
                        //no movement
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0.0);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                        //no movement (backwards) (could cause trouble during type conversions)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0.0);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                        //forward test
                    response = {0xDE, 0xAD, 0xBE, 0xEF, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0xDEADBEEF);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), (static_cast<double>(0xDEADBEEF) / ticks_per_rev) * wheel_radius);
                        //backwards/negative

                    response ={0xDE, 0xAD, 0xBE, 0xEF, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), -3735928559);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), static_cast<double>(-3735928559) / ticks_per_rev * wheel_radius);
                      //motor 2
                        //no movement
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), 0.0);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                        //no movement (backwards) (could cause trouble during type conversions)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), 0.0);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                        //forward test
                    response = {0xFE, 0xEB, 0xDA, 0xED, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), 0xFEEBDAED);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d(), static_cast<double>(0xFEEBDAED) / ticks_per_rev * wheel_radius);
                        //backwards/negative

                    response ={0xDE, 0xAD, 0xBE, 0xEF, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), -3735928559);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d(), static_cast<double>(-3735928559) / ticks_per_rev * wheel_radius);


                    //velocity tests
                      //motor 1
                        //still
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), 0.0);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(v(), 0.0);
                        //still(negative)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), 0.0);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(v(), 0.0);

                        //Forward
                    response = {0x0, 0x0, 0xFF, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), 0xFF00);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(v(),static_cast<double>(0xFF00) / ticks_per_rev * wheel_radius);
                        //backward
                    response = {0x0, 0x0, 0xFF, 0x0, 0x2};
                    testClaw->setcResponse(response);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), -65280);
                    EXPECT_DOUBLE_EQ(v(),static_cast<double>(-65280) / ticks_per_rev * wheel_radius);
                      //motor 2
                        //still
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0.0);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(v(), 0.0);
                        //still(negative)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0.0);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(v(), 0.0);

                        //Forward
                    response = {0x0, 0x0, 0xFF, 0xFF, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0xFFFF);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(v(),static_cast<double>(0xFFFF) / ticks_per_rev * wheel_radius);
                        //backward
                    response = {0x0, 0x0, 0xFF, 0x0, 0x2};
                    testClaw->setcResponse(response);
                    v=testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), -65280);
                    EXPECT_DOUBLE_EQ(v(),static_cast<double>(-65280) / ticks_per_rev * wheel_radius);

                    //plural tests
                    //readEncodersRaw, readEncoderVelocityRaw lack status/sign byte?
                    //TODO: a bit more responses
                      //stationary, +
                    response = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2=testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2(), d());
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2=testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0.0);
                    v=testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v(), 0.0);
                    v=testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v(), 0.0);



                      //stationary, -
                    response = {0x0, 0x0, 0x0, 0x0, 0,0 ,0, 0, 0x2, 0x2};
                    testClaw->setcResponse(response);

                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2=testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2=testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0.0);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], 0.0);

                    v=testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v(), 0.0);
                    v=testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v(), 0.0);

                      //Forward
                    response = {0x0, 0x0, 0xAB, 0x1, 0x0, 0x0, 0xAB, 0x1, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2=testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2(), d());
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2=testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0xAB01);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0xAB01);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], 0xAB01);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], 0xAB01);

                    v=testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v(), static_cast<double>(0xAB01) / ticks_per_rev * wheel_radius);
                    v=testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v(), static_cast<double>(0xAB01) / ticks_per_rev * wheel_radius);

                      //backwards
                    response = {0x0, 0x0, 0xAB, 0x1, 0x0, 0x0, 0xAB, 0x1, 0x2, 0x2};
                    testClaw->setcResponse(response);

                    d=testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2=testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2(), d());
                    d=testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2=testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], -43777);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], -43777);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], -43777);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], -43777);

                    v=testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v(), static_cast<double>(-43777) / ticks_per_rev * wheel_radius);
                    v=testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v(), static_cast<double>(-43777) / ticks_per_rev * wheel_radius);

                    //ensure proper transmission of setters
                    ticks = 0xABCD;
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM1, ticks);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    ticksc = static_cast<long>((response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);
                    //positive
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM2, ticks);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    ticksc = static_cast<long>((response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);
                    //negative ticks
                    ticks = -30000;
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM2, ticks);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    ticksc = static_cast<long>((response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);

                    d=100 * units::cm;
                    testClaw->setEncoder(Roboclaw::Motor::kM1, d);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    EXPECT_DOUBLE_EQ(d() * ticks_per_rev / wheel_radius, (response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);

                    testClaw->setEncoder(Roboclaw::Motor::kM2, d);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    EXPECT_DOUBLE_EQ(d() * ticks_per_rev / wheel_radius, (response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);

                    d=-100 * units::cm;
                    testClaw->setEncoder(Roboclaw::Motor::kM1, d);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    EXPECT_DOUBLE_EQ(d() * ticks_per_rev / wheel_radius, (response[2] << 8*3) + (response[3] << 8*2) + (response[4] << 8) + response[5]);


                    testClaw->resetEncoders();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kResetEnc));
                }
                TEST(RoboclawCore, Dynamics)
                {
                    FAIL() << "TODO: Implement Test";
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    /*
                    Implement setDynamics setPinFunctions
                    */
                }
                TEST(RoboclawCore, PID)
                {
                    FAIL() << "TODO: Implement Test";
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    /*
                    testClaw->
                    testClaw->
                    testClaw->
                    testClaw->
                    */
                }
                TEST(RoboclawCore, Misc)
                {
                    FAIL() << "TODO: Implement Test";
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    units::Temperature t;
                    /*
                    readtemperature, temp is in celsius.

                    response = {0x1, 0xF4};
                    testClaw->setcResponse(response);
                    t = testClaw->readTemperature();

                    testClaw->readVersion();
                    testClaw->getConfig();
                    testClaw->setConfig();
                    testClaw->setPinModes();
                    */
                }
            }
        }
    }
}
