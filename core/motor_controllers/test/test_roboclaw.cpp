#include "mock_roboclaw.hpp"
#include <motor_controllers/exceptions.hpp>

#include <memory>

#include <gtest/gtest.h>

#define ROBOCLAW_TEST_NEAR_ACCURACY_CM 0.01 // required precision in centimeters

namespace rip
{
    namespace motorcontrollers
    {
        namespace roboclaw
        {
            namespace tests
            {
                nlohmann::json j;
                using Roboclaw = mocks::MockRoboclaw;

                TEST(RoboclawCore, loadJson)
                {
                    j["name"] = "mock_roboclaw";
                    j["faking"] = "yep";
                    //test some types of variables that are not json obj
                    ASSERT_THROW(std::make_shared<Roboclaw>("strayTaco"), BadJson);
                    ASSERT_THROW(std::make_shared<Roboclaw>(42), BadJson);

                    //test if json null
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    units::Time t = 3000.0;
                    units::Distance d = .1;
                    j["address"] = 0x80; //uint8_t
                    j["timeout"] = t.to(units::ms);
                    j["ticks_per_rev"] = 100.0;//double
                    j["device"] = "/dev/ttyS0";
                    j["baudrate"] = 115200;

                    //test for missing entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    j["wheel_radius"] = "lol";
                    //tests for invalid values of entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j), BadJson);
                    j["wheel_radius"] = d.to(units::cm);
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
                    std::make_shared<Roboclaw>(j);
                    ASSERT_NO_THROW(std::make_shared<Roboclaw>(j));
                    //TODO: Reasonableness checks/nonfatal exceptions
                }

                TEST(RoboclawCore, DriveM1Forward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    //assuming valid roboclaw object at this point due to prior test
                    std::vector<uint8_t> response = {0xFF};
                    testClaw->setcResponse(response);
                    //CRC check failure throws command failure
                    testClaw->setBytes(55);
                    ASSERT_THROW(testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedForward), CommandFailure);
                    testClaw->setBytes(0);
                    //sends correct message: addr, 32 or 33, duty, CRC
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //final test: given proper conditions, does not throw an exception
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6u);
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
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6u);
                    //final test: given proper conditions, does not throw an exception
                    testClaw->drive(Roboclaw::Motor::kM1, Roboclaw::kFullSpeedBackward);
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
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 33);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6u);
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
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 32);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 6u);
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
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0xff);
                    ASSERT_EQ(testClaw->getLastCmd()[4], 0x7f);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0xff);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 8u);
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
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 34);
                    //concatenating elements 2 and 3 should give duty
                    ASSERT_EQ(testClaw->getLastCmd()[2], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[3], 0x1);
                    ASSERT_EQ(testClaw->getLastCmd()[4], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[5], 0x1);
                    //correct size of byte array
                    ASSERT_EQ(testClaw->getLastCmd().size(), 8u);
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
                    std::vector<uint8_t> response = {0, 80};
                    units::Voltage v;
                    units::Current a;
                    std::array<units::Voltage, 2> rv;
                    testClaw->setBytes(0);
                    //voltage out of range
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(-3 * units::V, 8 * units::V), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setLogicVoltages(6, 300), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(-3, 8), OutOfRange);
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(6, 300), OutOfRange);
                    //min voltage should be less than max voltage
                    ASSERT_THROW(std::make_shared<Roboclaw>(j)->setMainVoltages(18, 6), OutOfRange);

                    //set voltage, check to see if that is actual voltage
                    //verify proper command transmission
                    testClaw->setLogicVoltages(8 * units::V, 24 * units::V);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 58);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[2] << 8) + testClaw->getLastCmd()[3]), 80.0);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 240.0);
                    testClaw->setcResponse(response);
                    //initial voltage should be set to minimum voltage
                    v = testClaw->readLogicBatteryVoltage();
                    EXPECT_DOUBLE_EQ(v.to(units::V), 8.0);
                    //verify proper command transmission
                    testClaw->setMainVoltages(8 * units::V, 24 * units::V);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 57);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[2] << 8) + testClaw->getLastCmd()[3]), 80.0);
                    ASSERT_DOUBLE_EQ(static_cast<uint16_t>((testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 240.0);

                    response = {0, 80, 0, 240};
                    testClaw->setcResponse(response);
                    rv = testClaw->readMinMaxMainVoltages();
                    EXPECT_DOUBLE_EQ(rv[0].to(units::V), 8.0);
                    EXPECT_DOUBLE_EQ(rv[1].to(units::V), 24.0);
                    v = testClaw->readMainBatteryVoltage();

                    //readCurrent returns diff. than actual
                    //verify proper command transmission
                    /*
                    Send: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0, CRC(2 bytes)]
                    */
                    testClaw->setMaxCurrent(Roboclaw::Motor::kM1, 5 * units::A);
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], 133);
                    ASSERT_DOUBLE_EQ(static_cast<uint32_t>((testClaw->getLastCmd()[2] << 8 * 3) + (testClaw->getLastCmd()[3] << 8 * 2) + (testClaw->getLastCmd()[4] << 8) + testClaw->getLastCmd()[5]), 500.0);
                    //min current should always be 0
                    ASSERT_DOUBLE_EQ(static_cast<uint32_t>((testClaw->getLastCmd()[6] << 8 * 3) + (testClaw->getLastCmd()[7] << 8 * 2) + (testClaw->getLastCmd()[8] << 8) + testClaw->getLastCmd()[9]), 0.0);
                    //Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
                    response = {0, 0, 1, 0xF4};
                    testClaw->setcResponse(response);
                    ASSERT_LE(testClaw->readCurrent(Roboclaw::Motor::kM1), 5 * units::A);
                    ASSERT_EQ(testClaw->readMaxCurrent(Roboclaw::Motor::kM1), 5 * units::A);
                    ASSERT_EQ(testClaw->readCurrent(Roboclaw::Motor::kM1), 5 * units::A);
                    ASSERT_EQ(testClaw->readCurrents()[0], 5 * units::A);


                }
                TEST(RoboclawCore, Status)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    std::array<bool, 17> status;
                    //readstatus tests, status normal
                    response = {0, 0};
                    testClaw->setcResponse(response);
                    status = testClaw->readStatus();
                    EXPECT_TRUE(testClaw->readStatus(Roboclaw::Status::kNormal));
                    for (int i = 1; i < 17; i++)
                    {
                        EXPECT_FALSE(status[i]);
                    }
                    //opposite of above case, normal is false but all other statuses true
                    response = {0xFF, 0xFF};
                    testClaw->setcResponse(response);
                    status = testClaw->readStatus();
                    EXPECT_FALSE(testClaw->readStatus(Roboclaw::Status::kNormal));
                    for (int i = 1; i < 17; i++)
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

                TEST(RoboclawEncoder, ReadEncoderM1)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    //distance tests
                    //motor 1
                    //no movement
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0.0);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                    //no movement (backwards) (could cause trouble during type conversions)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), 0.0);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(d.to(units::cm), 0.0);

                    //forward test
                    response = {0x00, 0x00, 0xDE, 0xAD, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), (int32_t) 0x0000DEAD);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_NEAR(
                        d.to(units::cm),
                        (((1.0 * (int32_t)0x0000DEAD) / ticks_per_rev) * (wheel_radius.to(units::cm) * M_PI * 2)),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::cm) // absolute error allowed
                    );
                    //backwards/negative

                    response = {0x00, 0x00, 0xBE, 0xEF, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM1), (int32_t) 0x0000BEEF);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    EXPECT_NEAR(
                        d.to(units::cm),
                        (((1.0 * (int32_t)0x0000BEEF) / ticks_per_rev) * (wheel_radius.to(units::cm) * M_PI * 2)),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::cm) // absolute error allowed
                    );

                }

                TEST(RoboclawEncoder, ReadEncoderM2)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Distance d;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    //motor 2
                    //no movement
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), 0.0);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d(), 0.0);

                    //no movement (backwards) (could cause trouble during type conversions)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), 0.0);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(d.to(units::cm), 0.0);

                    //forward test
                    response = {0x00, 0x00, 0xF0, 0x0D, 0x0};
                    testClaw->setcResponse(response);

                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), (int32_t)0x0000F00D);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_NEAR(
                        d.to(units::cm),
                        (static_cast<double>(0x0000F00D) / ticks_per_rev) * wheel_radius.to(units::cm) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::cm) // absolute error allowed
                    );
                    //backwards/negative

                    response = {0x00, 0x00, 0xC0, 0x01, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderRaw(Roboclaw::Motor::kM2), (int32_t)0x0000C001);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    EXPECT_NEAR(
                        d.to(units::cm),
                        (static_cast<double>(0x0000C001) / ticks_per_rev) * wheel_radius.to(units::cm) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::cm) // absolute error allowed
                    );

                }

                TEST(RoboclawEncoder, ReadEncoderVelocityM1)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);//base unit meter
                    testClaw->setBytes(0);
                    //velocity tests
                    //motor 1
                    //still
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), 0.0);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(v.to(units::m / units::s), 0.0);
                    //still(negative)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), 0.0);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(v.to(units::m / units::s), 0.0);

                    //Forward
                    response = {0x00, 0x00, 0xCA, 0xFE, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), (int32_t)0xCAFE);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        (static_cast<double>(0xCAFE) / ticks_per_rev) * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );
                    //backward
                    response = {0xFF, 0xFF, 0xF1, 0x00, 0x2}; // negative values (twos complement)
                    testClaw->setcResponse(response);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM1);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM1), (int32_t) - 0x0F00);
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        (static_cast<double>(-0x0F00) / ticks_per_rev) * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );


                }

                TEST(RoboclawEncoder, ReadEncoderVelocityM2)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);//base unit meter
                    testClaw->setBytes(0);

                    //motor 2
                    //still
                    response = {0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0.0);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(v.to(units::m / units::s), 0.0);
                    //still(negative)
                    response = {0x0, 0x0, 0x0, 0x0, 2};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0.0);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(v.to(units::m / units::s), 0.0);

                    //Forward
                    response = {0x0, 0x0, 0xFF, 0xFF, 0x0};
                    testClaw->setcResponse(response);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), 0xFFFF);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        (static_cast<double>(0xFFFF) / ticks_per_rev) * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );
                    //backward
                    response = {0xFF, 0xFF, 0xF3, 0x00, 0x2}; // twos complement for negatives
                    testClaw->setcResponse(response);
                    v = testClaw->readEncoderVelocity(Roboclaw::Motor::kM2);
                    EXPECT_DOUBLE_EQ(testClaw->readEncoderVelocityRaw(Roboclaw::Motor::kM2), -0x0D00);
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        (static_cast<double>(-0x0D00) / ticks_per_rev) * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );

                }

                TEST(RoboclawEncoder, PluralStationary)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d, d2;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    //stationary, +
                    response = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
                    testClaw->setcResponse(response);

                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2 = testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2 = testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0.0);
                    v = testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v.to(units::cm / units::s), 0.0);
                    v = testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v.to(units::cm / units::s), 0.0);



                    //stationary, -
                    response = {0x0, 0x0, 0x0, 0x0, 0, 0 , 0, 0, 0x2, 0x2};
                    testClaw->setcResponse(response);

                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2 = testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));

                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2 = testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0.0);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], 0.0);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], 0.0);

                    v = testClaw->readEncodersVelocity()[0];
                    EXPECT_DOUBLE_EQ(v.to(units::cm / units::s), 0.0);
                    v = testClaw->readEncodersVelocity()[1];
                    EXPECT_DOUBLE_EQ(v.to(units::cm / units::s), 0.0);

                }

                TEST(RoboclawEncoder, PluralForward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d, d2;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    //Forward
                    response = {0x0, 0x0, 0xAB, 0x1, 0x0, 0x0, 0xAB, 0x1, 0x0, 0x0};
                    testClaw->setcResponse(response);
                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2 = testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2(), d());
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2 = testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2(), d());

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], 0xAB01);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 0xAB01);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], 0xAB01);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], 0xAB01);

                    v = testClaw->readEncodersVelocity()[0];
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        static_cast<double>(0xAB01) / ticks_per_rev * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );
                    v = testClaw->readEncodersVelocity()[1];
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        static_cast<double>(0xAB01) / ticks_per_rev * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );

                }

                TEST(RoboclawEncoder, PluralBackward)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d, d2;
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    //backwards
                    response =
                    {
                        0xFF, 0xFF, 0x54, 0xFF, // this one negative
                        0x00, 0x00, 0xAB, 0x01,
                        0x02, 0x02
                    };
                    testClaw->setcResponse(response);

                    d = testClaw->readEncoder(Roboclaw::Motor::kM1);
                    d2 = testClaw->readEncoders()[0];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));
                    d = testClaw->readEncoder(Roboclaw::Motor::kM2);
                    d2 = testClaw->readEncoders()[1];
                    EXPECT_DOUBLE_EQ(d2.to(units::cm), d.to(units::cm));

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[0], -43777);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersRaw()[1], 43777);

                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[0], -43777);
                    EXPECT_DOUBLE_EQ(testClaw->readEncodersVelocityRaw()[1], 43777);

                    v = testClaw->readEncodersVelocity()[0];
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        static_cast<double>(-43777) / ticks_per_rev * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );
                    v = testClaw->readEncodersVelocity()[1];
                    EXPECT_NEAR(
                        v.to(units::m / units::s),
                        static_cast<double>(43777) / ticks_per_rev * wheel_radius.to(units::m) * (M_PI * 2),
                        units::Length(units::cm * ROBOCLAW_TEST_NEAR_ACCURACY_CM).to(units::m)
                    );

                }

                TEST(RoboclawEncoder, setTicksRaw)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    units::Velocity v;
                    units::Distance d, d2;
                    units::Length wheel_radius(units::m * 0.04);
                    int ticks, ticksc;
                    testClaw->setBytes(0);

                    //ensure proper transmission of setters
                    ticks = 0xABCD;
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM1, ticks);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    ticksc = static_cast<long>((response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);
                    //positive
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM2, ticks);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    ticksc = static_cast<long>((response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);
                    //negative ticks
                    ticks = -30000;
                    testClaw->setEncoderRaw(Roboclaw::Motor::kM2, ticks);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    ticksc = static_cast<long>((response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]);
                    ASSERT_EQ(ticks, ticksc);


                    //reset encoder
                    testClaw->resetEncoders();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kResetEnc));

                }
                TEST(RoboclawEncoder, setTicks)
                {

                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    const double ticks_per_rev = 360.0;
                    units::Velocity v;
                    units::Distance d, d2;
                    units::Length wheel_radius(units::m * 0.04);
                    testClaw->setBytes(0);

                    d = 100 * units::cm;
                    testClaw->setEncoder(Roboclaw::Motor::kM1, d);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    EXPECT_DOUBLE_EQ(
                        static_cast<int32_t>(d.to(units::cm) * ticks_per_rev / wheel_radius.to(units::cm) / (M_PI * 2)),
                        (response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]
                    );

                    testClaw->setEncoder(Roboclaw::Motor::kM2, d);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2EncCount));
                    EXPECT_DOUBLE_EQ(
                        static_cast<int32_t>(d.to(units::cm) * ticks_per_rev / wheel_radius.to(units::cm) / (M_PI * 2)),
                        (response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]
                    );

                    d = -100 * units::cm;
                    testClaw->setEncoder(Roboclaw::Motor::kM1, d);
                    response = testClaw->getLastCmd();
                    ASSERT_EQ(testClaw->getLastCmd()[0], 0x80);
                    ASSERT_EQ(testClaw->getLastCmd()[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1EncCount));
                    EXPECT_DOUBLE_EQ(
                        static_cast<int32_t>(d.to(units::cm) * ticks_per_rev / wheel_radius.to(units::cm) / (M_PI * 2)),
                        (response[2] << 8 * 3) + (response[3] << 8 * 2) + (response[4] << 8) + response[5]
                    );

                }

                TEST(RoboclawDynamics, Class)
                {
                    std::shared_ptr<MotorDynamics> dynamics(new MotorDynamics);
                    units::Distance d = 1.0, d2;
                    units::Velocity v = 1.0, v2;//default unit m/s?
                    units::Acceleration a = 1.0, a2;

                    //returns nullptrs when instantiated
                    ASSERT_EQ(dynamics->getDistance(), nullptr);
                    ASSERT_EQ(dynamics->getSpeed(), nullptr);
                    ASSERT_EQ(dynamics->getAcceleration(), nullptr);
                    ASSERT_EQ(dynamics->getDeceleration(), nullptr);
                    ASSERT_EQ(dynamics->getDType(), MotorDynamics::DType::kNone);

                    dynamics->setDistance(d);
                    dynamics->setSpeed(v);
                    dynamics->setAcceleration(a);
                    dynamics->setDeceleration(a);

                    d2 = *(dynamics->getDistance());
                    ASSERT_DOUBLE_EQ(d2(), d());
                    v2 = *(dynamics->getSpeed());
                    ASSERT_DOUBLE_EQ(v2(), v());
                    a2 = *(dynamics->getAcceleration());
                    ASSERT_DOUBLE_EQ(a2(), a());
                    a2 = *(dynamics->getDeceleration());
                    ASSERT_DOUBLE_EQ(a2(), a());

                    ASSERT_EQ(dynamics->getDType(), MotorDynamics::DType::kSpeedAccelDecelDist);
                    //negative values
                    d = -1;
                    a = -1;
                    ASSERT_THROW(dynamics->setDistance(d), OutOfRange);
                    ASSERT_THROW(dynamics->setAcceleration(a), OutOfRange);
                    ASSERT_THROW(dynamics->setDeceleration(a), OutOfRange);

                }
                TEST(RoboclawDynamics, setDynamicsM1)
                {
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    std::vector<uint8_t> message;
                    testClaw->setBytes(0);
                    std::shared_ptr<MotorDynamics> dynamics(new MotorDynamics);


                    units::Distance d = 1.0;
                    units::Velocity v = 99.5;//default unit m/s?
                    units::Acceleration a = 45.0;
                    //dynamics empty
                    /*
                    Notes on setDynamics: speed is only signed parameter, dist, acc, dec,
                    are unsigned as defined by Roboclaw manual.
                    */
                    EXPECT_NO_THROW(testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics));
                    //ensure proper transmission
                    //motor 1
                    //speed (+)
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1Speed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed (-)
                    v = -1;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1Speed));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed 0
                    v = 0;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1Speed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(0.0, (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & acceleration(+)
                    v = 1;
                    dynamics->setSpeed(v);
                    dynamics->setAcceleration(a);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1SpeedAccel));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & distance(+)
                    dynamics->setDistance(d);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1SpeedAccelDist));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((d / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[6] << 8 * 3) + (message[7] << 8 * 2) + (message[8] << 8) + message[9]);

                    //speed & acceleration & distance & deceleration(+)
                    dynamics->setDeceleration(a);
                    testClaw->setDynamics(Roboclaw::Motor::kM1, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM1SpeedAccelDeccelPos));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[10] << 8 * 3) + (message[11] << 8 * 2) + (message[12] << 8) + message[13]);



                }

                TEST(RoboclawDynamics, setDynamicsM2)
                {
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    std::vector<uint8_t> message;
                    testClaw->setBytes(0);
                    std::shared_ptr<MotorDynamics> dynamics(new MotorDynamics);

                    units::Distance d = 1.0;
                    units::Velocity v = 99.5;//default unit m/s?
                    units::Acceleration a = 45.0;
                    //dynamics empty
                    /*
                    Notes on setDynamics: speed is only signed parameter, dist, acc, dec,
                    are unsigned as defined by Roboclaw manual.
                    */
                    EXPECT_NO_THROW(testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics));
                    //ensure proper transmission
                    //motor 1
                    //speed (+)
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2Speed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed (-)
                    v = -1;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2Speed));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed 0
                    v = 0;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2Speed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(0.0, (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & acceleration(+)
                    v = 1;
                    dynamics->setSpeed(v);
                    dynamics->setAcceleration(a);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2SpeedAccel));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & distance(+)
                    dynamics->setDistance(d);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2SpeedAccelDist));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((d / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[6] << 8 * 3) + (message[7] << 8 * 2) + (message[8] << 8) + message[9]);

                    //speed & acceleration & distance & deceleration(+)
                    dynamics->setDeceleration(a);
                    testClaw->setDynamics(Roboclaw::Motor::kM2, *dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kM2SpeedAccelDeccelPos));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[10] << 8 * 3) + (message[11] << 8 * 2) + (message[12] << 8) + message[13]);



                }


                TEST(RoboclawDynamics, setDynamicsPlural)
                {
                    double ticks_per_rev = 360.0;
                    units::Length wheel_radius(units::m * 0.04);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    std::vector<uint8_t> message;
                    testClaw->setBytes(0);
                    std::shared_ptr<MotorDynamics> dynamics(new MotorDynamics);

                    units::Distance d = 1.0;
                    units::Velocity v = 99.5;//default unit m/s?
                    units::Acceleration a = 45.0;
                    //dynamics empty
                    /*
                    Notes on setDynamics: speed is only signed parameter, dist, acc, dec,
                    are unsigned as defined by Roboclaw manual.
                    */
                    EXPECT_NO_THROW(testClaw->setDynamics(*dynamics));
                    //ensure proper transmission
                    //motors 1&2
                    //speed (+)
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed (-)
                    v = -1;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeed));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((v / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed 0
                    v = 0;
                    dynamics->setSpeed(v);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeed));
                    //speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius)() * m_ticks_per_rev);
                    EXPECT_DOUBLE_EQ(0.0, (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & acceleration(+)
                    v = 1;
                    dynamics->setSpeed(v);
                    dynamics->setAcceleration(a);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeedAccel));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[2] << 8 * 3) + (message[3] << 8 * 2) + (message[4] << 8) + message[5]);
                    //speed & distance(+)
                    dynamics->setDistance(d);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeedAccelDist));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((d / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[6] << 8 * 3) + (message[7] << 8 * 2) + (message[8] << 8) + message[9]);

                    //speed & acceleration & distance & deceleration(+)
                    dynamics->setDeceleration(a);
                    testClaw->setDynamics(*dynamics);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kMixedSpeedAccelDeccelPos));
                    EXPECT_DOUBLE_EQ(static_cast<int32_t>((a / wheel_radius / (M_PI * 2))() * ticks_per_rev),
                                     (message[10] << 8 * 3) + (message[11] << 8 * 2) + (message[12] << 8) + message[13]);



                }

                TEST(RoboclawPID, VelocityM1)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response, message;
                    testClaw->setBytes(0);

                    std::shared_ptr<VelocityPIDParameters> vparams(new VelocityPIDParameters);

                    //given response, received properly
                    //p,i,d,qpps, 4 bytes each
                    response = {0x0E, 0xAD, 0xBE, 0xEF,//d
                                0x12, 0x34, 0x56, 0x78, //p
                                0xAB, 0xCD, 0xEF, 0x00, //i
                                0x0, 0x01, 0x00, 0x00
                               }; //qpps
                    testClaw->setcResponse(response);
                    *vparams = testClaw->readVelocityPID(Roboclaw::Motor::kM1);
                    //should be > 0
                    EXPECT_GE(vparams->kp, 0);
                    EXPECT_GE(vparams->ki, 0);
                    EXPECT_GE(vparams->kd, 0);

                    EXPECT_FLOAT_EQ(vparams->kp, static_cast<float>(0x0EADBEEF) / 65536);
                    EXPECT_FLOAT_EQ(vparams->ki, static_cast<float>(0x12345678) / 65536);
                    EXPECT_FLOAT_EQ(vparams->kd, static_cast<float>(0xABCDEF00) / 65536);
                    EXPECT_EQ(vparams->qpps, 0x00010000);

                    //validate transmissions
                    testClaw->setVelocityPID(Roboclaw::Motor::kM1, *vparams);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1PID));
                    //d
                    ASSERT_NEAR(static_cast<uint32_t>((message[2] << 8 * 3) + (message[3] << 8 * 2) +
                                                      (message[4] << 8) + message[5]), 0xABCDEF00, 20);
                    //p (B/c of float, the int is close but not = the original value)
                    ASSERT_NEAR(static_cast<uint32_t>((message[6] << 8 * 3) + (message[7] << 8 * 2) +
                                                      (message[8] << 8) + message[9]), 0x0EADBEEF, 20);
                    //i
                    ASSERT_NEAR((message[10] << 8 * 3) + (message[11] << 8 * 2) +
                                (message[12] << 8) + message[13], 0x12345678, 20);
                    //qpps
                    ASSERT_EQ((message[14] << 8 * 3) + (message[15] << 8 * 2) +
                              (message[16] << 8) + message[17], 0x10000);




                }

                TEST(RoboclawPID, VelocityM2)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response, message;
                    testClaw->setBytes(0);
                    std::shared_ptr<VelocityPIDParameters> vparams(new VelocityPIDParameters);

                    //given response, received properly
                    //p,i,d,qpps, 4 bytes each
                    response = {0xDE, 0xAD, 0xBE, 0xEF,//p
                                0x12, 0x34, 0x56, 0x78, //i
                                0xAB, 0xCD, 0xEF, 0x00, //d
                                0x0, 0x01, 0x00, 0x00
                               }; //qpps
                    testClaw->setcResponse(response);
                    *vparams = testClaw->readVelocityPID(Roboclaw::Motor::kM2);
                    //should be > 0
                    EXPECT_GE(vparams->kp, 0);
                    EXPECT_GE(vparams->ki, 0);
                    EXPECT_GE(vparams->kd, 0);
                    EXPECT_FLOAT_EQ(vparams->kp, static_cast<float>(0xDEADBEEF) / 65536);
                    EXPECT_FLOAT_EQ(vparams->ki, static_cast<float>(0x12345678) / 65536);
                    EXPECT_FLOAT_EQ(vparams->kd, static_cast<float>(0xABCDEF00) / 65536);
                    EXPECT_EQ(vparams->qpps, 0x00010000);

                    //validate transmissions
                    testClaw->setVelocityPID(Roboclaw::Motor::kM2, *vparams);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kSetM2PID));
                    //d
                    ASSERT_NEAR(static_cast<uint32_t>((message[2] << 8 * 3) + (message[3] << 8 * 2) +
                                                      (message[4] << 8) + message[5]), 0xABCDEF00, 20);
                    //p
                    ASSERT_NEAR(static_cast<uint32_t>((message[6] << 8 * 3) + (message[7] << 8 * 2) +
                                                      (message[8] << 8) + message[9]), 0xDEADBEEF, 20);
                    //i
                    ASSERT_NEAR(static_cast<uint32_t>((message[10] << 8 * 3) + (message[11] << 8 * 2) +
                                                      (message[12] << 8) + message[13]), 0x12345678, 20);
                    //qpps
                    ASSERT_EQ((message[14] << 8 * 3) + (message[15] << 8 * 2) +
                              (message[16] << 8) + message[17], 0x10000);



                }

                TEST(RoboclawPID, PositionM1)
                {
                    /*
                    Receive: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte),
                    MinPos(4 byte), MaxPos(4 byte), CRC(2 bytes)]
                    */
                    std::shared_ptr<PositionPIDParameters> pparams(new PositionPIDParameters);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response, message;
                    testClaw->setBytes(0);

                    //values should be initialized to 0
                    EXPECT_FLOAT_EQ(pparams->kp, 0);
                    EXPECT_FLOAT_EQ(pparams->ki, 0);
                    EXPECT_FLOAT_EQ(pparams->kd, 0);
                    EXPECT_EQ(pparams->kiMax, 0u);
                    EXPECT_EQ(pparams->deadzone, 0u);
                    EXPECT_EQ(pparams->min, 0u);
                    EXPECT_EQ(pparams->max, 0u);

                    response = {0xDE, 0xAD, 0xBE, 0xEF,//p
                                0x12, 0x34, 0x56, 0x78, //i
                                0xAB, 0xCD, 0xEF, 0x00, //d
                                0x0, 0x11, 0x00, 0x00, //maxI
                                0x01, 0x35, 0x79, 0xBD, //deadzone
                                0x00, 0x00, 0x11, 0x00, //minpos
                                0x0F, 0xFF, 0xFF, 0xFF
                               }; //maxpos
                    testClaw->setcResponse(response);

                    *pparams = testClaw->readPositionPID(Roboclaw::Motor::kM2);
                    //value should be greater than 0
                    EXPECT_GE(pparams->kp, 0);
                    EXPECT_GE(pparams->ki, 0);
                    EXPECT_GE(pparams->kd, 0);
                    EXPECT_GE(pparams->kiMax, 0u);
                    EXPECT_GE(pparams->deadzone, 0u);
                    EXPECT_GE(pparams->min, 0u);
                    EXPECT_GE(pparams->max, 0u);

                    //floats are trash
                    ASSERT_NEAR(pparams->kp, static_cast<float>(0xDEADBEEF) / 1024, 20);
                    EXPECT_NEAR(pparams->ki, static_cast<float>(0x12345678) / 1024, 20);
                    ASSERT_NEAR(pparams->kd, static_cast<float>(0xABCDEF00) / 1024, 20);
                    EXPECT_EQ(pparams->kiMax, 0x00110000);
                    EXPECT_EQ(pparams->deadzone, 0x013579BD);
                    EXPECT_EQ(pparams->min, 0x00001100);
                    EXPECT_EQ(pparams->max, 0x0FFFFFFF);

                    //validate transmissions
                    testClaw->setPositionPID(Roboclaw::Motor::kM1, *pparams);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1PosPID));
                    //d
                    EXPECT_NEAR(static_cast<uint32_t>((message[2] << 8 * 3) + (message[3] << 8 * 2) +
                                                      (message[4] << 8) + message[5]), 0xABCDEF00, 20);
                    //p
                    ASSERT_NEAR(static_cast<uint32_t>((message[6] << 8 * 3) + (message[7] << 8 * 2) +
                                                      (message[8] << 8) + message[9]), 0xDEADBEEF, 20);
                    //i
                    ASSERT_NEAR((message[10] << 8 * 3) + (message[11] << 8 * 2) +
                                (message[12] << 8) + message[13], 0x12345678, 20);
                    //kiMax
                    ASSERT_EQ((message[14] << 8 * 3) + (message[15] << 8 * 2) +
                              (message[16] << 8) + message[17], 0x110000);
                    //deadzone
                    ASSERT_EQ(static_cast<uint32_t>((message[18] << 8 * 3) + (message[19] << 8 * 2) +
                                                    (message[20] << 8) + message[21]), 0x13579BD);
                    //min
                    ASSERT_EQ((message[22] << 8 * 3) + (message[23] << 8 * 2) +
                              (message[24] << 8) + message[25], 0x1100);
                    //max
                    ASSERT_EQ((message[26] << 8 * 3) + (message[27] << 8 * 2) +
                              (message[28] << 8) + message[29], 0x0FFFFFFF);

                }

                TEST(RoboclawPID, PositionM2)
                {
                    std::shared_ptr<PositionPIDParameters> pparams(new PositionPIDParameters);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response, message;
                    testClaw->setBytes(0);

                    /*
                    Receive: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte),
                    MinPos(4 byte), MaxPos(4 byte), CRC(2 bytes)]
                    */

                    response = {0xDE, 0xAD, 0xBE, 0xEF,//p
                                0x12, 0x34, 0x56, 0x78, //i
                                0xAB, 0xCD, 0xEF, 0x00, //d
                                0x0, 0x01, 0x00, 0x00, //maxI
                                0x01, 0x35, 0x79, 0xBD, //deadzone
                                0x00, 0x00, 0x11, 0x00, //minpos
                                0x0F, 0xFF, 0xFF, 0xFF
                               }; //maxpos
                    testClaw->setcResponse(response);


                    *pparams = testClaw->readPositionPID(Roboclaw::Motor::kM2);
                    //value should be greater than 0
                    EXPECT_GE(pparams->kp, 0);
                    EXPECT_GE(pparams->ki, 0);
                    EXPECT_GE(pparams->kd, 0);
                    EXPECT_GE(pparams->kiMax, 0u);
                    EXPECT_GE(pparams->deadzone, 0u);
                    EXPECT_GE(pparams->min, 0u);
                    EXPECT_GE(pparams->max, 0u);

                    ASSERT_NEAR(pparams->kp, static_cast<float>(0xDEADBEEF) / 1024, 20);
                    EXPECT_NEAR(pparams->ki, static_cast<float>(0x12345678) / 1024, 20);
                    ASSERT_NEAR(pparams->kd, static_cast<float>(0xABCDEF00) / 1024, 20);
                    EXPECT_EQ(pparams->kiMax, 0x00010000);
                    EXPECT_EQ(pparams->deadzone, 0x013579BD);
                    EXPECT_EQ(pparams->min, 0x00001100);
                    EXPECT_EQ(pparams->max, 0x0FFFFFFF);

                    //validate transmissions
                    testClaw->setPositionPID(Roboclaw::Motor::kM1, *pparams);
                    message = testClaw->getLastCmd();
                    ASSERT_EQ(message[0], 0x80);
                    ASSERT_EQ(message[1], static_cast<uint8_t>(Roboclaw::Command::kSetM1PosPID));
                    //d
                    ASSERT_NEAR(static_cast<uint32_t>((message[2] << 8 * 3) + (message[3] << 8 * 2) +
                                                      (message[4] << 8) + message[5]), 0xABCDEF00, 20);
                    //p
                    ASSERT_NEAR(static_cast<uint32_t>((message[6] << 8 * 3) + (message[7] << 8 * 2) +
                                                      (message[8] << 8) + message[9]), 0xDEADBEEF, 20);
                    //i
                    ASSERT_NEAR(static_cast<uint32_t>((message[10] << 8 * 3) + (message[11] << 8 * 2) +
                                                      (message[12] << 8) + message[13]), 0x12345678, 20);
                    //kiMax
                    ASSERT_EQ((message[14] << 8 * 3) + (message[15] << 8 * 2) +
                              (message[16] << 8) + message[17], 0x10000);
                    //deadzone
                    ASSERT_EQ((message[18] << 8 * 3) + (message[19] << 8 * 2) +
                              (message[20] << 8) + message[21], 0x13579BD);
                    //min
                    ASSERT_EQ((message[22] << 8 * 3) + (message[23] << 8 * 2) +
                              (message[24] << 8) + message[25], 0x1100);
                    //max
                    ASSERT_EQ((message[26] << 8 * 3) + (message[27] << 8 * 2) +
                              (message[28] << 8) + message[29], 0x0FFFFFFF);



                }

                TEST(RoboclawConfig, ClassSetters)
                {
                    std::shared_ptr<Config> config(new Config);
                    //comm mode setters work properly
                    uint16_t mask;
                    for (mask = 0; mask < 3; mask++)
                    {
                        config->setCommMode(static_cast<Config::CommMode>(mask));
                        ASSERT_EQ((config->get() & mask), mask);
                    }
                    //battery setters work properly
                    for (mask = 0; mask < 4 * 8; mask += 4)
                    {
                        config->setBatteryMode(static_cast<Config::BatteryMode>(mask));
                        EXPECT_EQ((config->get() & mask), mask);
                    }

                    config->setCommMode(Config::CommMode::kAnalog);
                    ASSERT_THROW(config->getBaudRate(), InvalidCommMode);
                    config->setCommMode(Config::CommMode::kPacketSerial);
                    //Baudrate setters work properly
                    for (mask = 0; mask < 8 * 32; mask += 32)
                    {
                        config->setBaudRate(static_cast<Config::BaudRate>(mask));
                        EXPECT_EQ((config->get() & mask), mask);
                    }
                    //packet addr setters work properly
                    for (mask = 0; mask < 8 * 256; mask += 256)
                    {
                        config->setPacketAddress(static_cast<Config::PacketAddress>(mask));
                        EXPECT_EQ((config->get() & mask), mask);
                    }

                    config->setSwapButtons(1);
                    mask = 0x4000;
                    EXPECT_EQ(config->get() & mask, mask);

                    config->setSwapEncoders(1);
                    mask = 0x2000;
                    EXPECT_EQ(config->get() & mask, mask);
                }
                TEST(RoboclawConfig, ClassGetters)
                {
                    std::shared_ptr<Config> config(new Config);

                    //packet serial
                    uint16_t cfg = 0x80A7;
                    config->set(cfg);
                    ASSERT_EQ(static_cast<uint16_t>(config->getCommMode()),
                              static_cast<uint16_t>(Config::CommMode::kPacketSerial));
                    EXPECT_EQ(static_cast<uint16_t>(config->getPacketAddress()),
                              static_cast<uint16_t>(Config::PacketAddress::k80));
                    EXPECT_EQ(static_cast<uint16_t>(config->getBatteryMode()),
                              static_cast<uint16_t>(Config::BatteryMode::kAuto));
                    EXPECT_EQ(static_cast<uint16_t>(config->getBaudRate()),
                              static_cast<uint16_t>(Config::BaudRate::k115200));

                    //RC + battery off
                    cfg = 0;
                    config->set(cfg);
                    ASSERT_EQ(static_cast<uint16_t>(config->getCommMode()),
                              static_cast<uint32_t>(Config::CommMode::kRC));
                    ASSERT_THROW(config->getPacketAddress(), InvalidCommMode);
                    ASSERT_THROW(config->getBaudRate(), InvalidCommMode);
                    EXPECT_EQ(static_cast<uint16_t>(config->getBatteryMode()),
                              static_cast<uint32_t>(Config::BatteryMode::kOff));
                }
                TEST(RoboclawConfig, Other)
                {
                    std::shared_ptr<Config> config(new Config);
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response, message;
                    testClaw->setBytes(0);
                    response = {0x80, 0xA7};
                    testClaw->setcResponse(response);
                    *config = testClaw->getConfig();
                    EXPECT_EQ(config->get(), 0x80A7);
                    //validate transmission
                    config->set(0x80A7);
                    testClaw->setConfig(*config);
                    message = testClaw->getLastCmd();
                    EXPECT_EQ(message[0], 0x80);
                    EXPECT_EQ(message[1], 98);
                    EXPECT_EQ((message[2] << 8) + message[3], 0x80A7);
                }

                TEST(RoboclawMisc, Temperature)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);
                    units::Temperature t;
                    /*
                    readtemperature, temp is in celsius.
                    default unit is kelvin, but 1K = 1C
                    */
                    response = {0x1, 0xF4};
                    testClaw->setcResponse(response);
                    t = testClaw->readTemperature();
                    EXPECT_DOUBLE_EQ(t(), 50.0);

                }
                TEST(RoboclawMisc, readBuffers)
                {
                    std::shared_ptr<Roboclaw> testClaw(new Roboclaw);
                    std::vector<uint8_t> response;
                    testClaw->setBytes(0);

                    response = {0x80, 0x3F};
                    testClaw->setcResponse(response);
                    EXPECT_EQ(testClaw->readBufferLens()[0], 0x80);
                    EXPECT_EQ(testClaw->readBufferLens()[1], 0x3F);
                }
            }
        }
    }
}
