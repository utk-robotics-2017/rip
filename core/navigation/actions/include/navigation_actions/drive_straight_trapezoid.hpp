#ifndef DRIVE_STRAIGHT_TRAPEZOID_HPP
#define DRIVE_STRAIGHT_TRAPEZOID_HPP

#include <json.hpp>

#include <framework/action.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveStraightTrapezoid : public framework::Action
            {
            public:

                /**
                * Returns whether or not the action has finished execution.
                */
                virtual bool isFinished() override
                {
                    // todo
                }

                /**
                 * Iteratively called until {@see Action#isFinished()} returns true
                 */
                virtual void update(nlohmann::json& state) override
                {
                    // todo
                }

                /**
                 * Run once before the main code
                 */
                virtual void setup(nlohmann::json& state) override
                {
                    // todo
                }

                /**
                 * Run once after finished
                 */
                virtual void teardown(nlohmann::json& state) override
                {
                    // todo
                }

            private:
            };
        }
    }
}

#endif // DRIVE_STRAIGHT_TRAPEZOID_HPP
