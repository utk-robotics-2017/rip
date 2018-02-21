#ifndef DRIVE_ARC_HPP
#define DRIVE_ARC_HPP

#include <json.hpp>

#include <framework/action.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveArc : public framework::Action
            {
            public:



                /**
                * Returns whether or not the action has finished execution.
                */
                virtual bool isFinished() override;

                /**
                 * Iteratively called until {@see Action#isFinished()} returns true
                 */
                virtual void update(nlohmann::json& state) override;

                /**
                 * Run once before the main code
                 */
                virtual void setup(nlohmann::json& state) override;

                /**
                 * Run once after finished
                 */
                virtual void teardown(nlohmann::json& state) override;

            private:
            };
        }
    }
}

#endif // DRIVE_ARC_HPP
