#include "instant_action.hpp"

namespace rip
{
    namespace scheduler
    {
        InstantAction::InstantAction(std::string name)
            : Action(name)
        {}

        void isFinished() override
        {
            return true;
        }
    }
}