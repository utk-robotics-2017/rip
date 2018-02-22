#include "framework/parallel_action.hpp"

namespace rip
{
    namespace framework
    {
        ParallelAction::ParallelAction(const std::string& name, const std::vector<std::shared_ptr<Action> >& actions)
            : Action(name)
            , m_actions(actions)
        {}

        bool ParallelAction::isFinished()
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                if (!action->isFinished())
                {
                    return false;
                }
            }
            return true;
        }

        void ParallelAction::update(nlohmann::json& state)
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                action->update(state);
            }
        }

        void ParallelAction::setup(nlohmann::json& state)
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                action->setup(state);
            }
        }

        void ParallelAction::teardown(nlohmann::json& state)
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                action->teardown(state);
            }
        }
    }

}
