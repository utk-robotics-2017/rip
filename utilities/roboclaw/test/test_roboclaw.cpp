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
            //using Roboclaw = mocks::MockRoboclaw;

            namespace tests
            {
                TEST(RoboclawCore, loadJson)
                {
                    //test some types of variables that are not json obj

                    ASSERT_THROW(std::make_shared<Roboclaw>("strayTaco", 1), BadJson);
                    ASSERT_THROW(std::make_shared<Roboclaw>(42, 1), BadJson);

                    //test if json null
                    nlohmann::json j;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);
                    units::Time t = 3.0;
                    units::Distance d = .1;
                    j["address"] = 0x80; //uint8_t
                    j["timeout"] = t.to(units::ms);
                    j["ticks_per_rev"] = 100.0;//double
                    //test for missing entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);
                    j["wheel_radius"] = "lol";
                    //tests for invalid values of entries
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);
                    j["wheel_radius"] = d.to(units::cm);
                    j["address"] = 267;//255 is largest possible value
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), OutOfRange);
                    j["address"] = 0x80;
                    j["timeout"] = -3.0;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), OutOfRange);
                    j["timeout"] = 10.0;
                    j["ticks_per_rev"] = j;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);
                    //test extra entries
                    j["extra"] = 1;
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);
                    j.erase("extra");
                    //verify entry types are correct
                    ASSERT_THROW(std::make_shared<Roboclaw>(j,1), BadJson);

                    j["timeout"] = t.to(units::ms);
                    j["ticks_per_rev"] = 100.0;
                    j["wheel_radius"] = d.to(units::cm);
                    //given valid parameters, does not throw an exception
                    ASSERT_NO_THROW(std::make_shared<Roboclaw>(j,1));
                    //TODO: Reasonableness checks/nonfatal exceptions
                }


                TEST(RoboclawCore, DriveM1Forward)
                {
                    FAIL() << "TODO: Implement Test";
                }

                TEST(RoboclawCore, DriveM1Backward)
                {
                    FAIL() << "TODO: Implement Test";
                }

                TEST(RoboclawCore, DriveM2Forward)
                {
                    FAIL() << "TODO: Implement Test";
                }

                TEST(RoboclawCore, DriveM2Backward)
                {
                    FAIL() << "TODO: Implement Test";
                }
            }
        }
    }
}
