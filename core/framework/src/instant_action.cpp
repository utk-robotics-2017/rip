#include "framework/instant_action.hpp"

namespace rip
{

    namespace framework
    {
        InstantAction::InstantAction(const std::string& name)
            : Action(name)
        {}

        bool InstantAction::isFinished()
        {
            return true;
        }

        void InstantAction::update(nlohmann::json& state) {}

        void InstantAction::setup(nlohmann::json& state)
        {
            runOnce();
        }

        void InstantAction::teardown(nlohmann::json& state) {}
    }
}
