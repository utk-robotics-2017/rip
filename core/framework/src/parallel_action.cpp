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
            return m_actions.size() == m_finished.size();
        }

        void ParallelAction::update(nlohmann::json& state)
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                if (m_finished.find(action) != m_finished.end())
                {
                    continue;
                }
                if (action->isFinished())
                {
                    m_finished.insert(action);
                    action->teardown(state);
                }
                else
                {
                    action->update(state);
                }
            }
        }

        void ParallelAction::setup(nlohmann::json& state)
        {
            for (std::shared_ptr<Action> action : m_actions)
            {
                action->setup(state);
            }
        }

        // all actions should have already been torn down
        void ParallelAction::teardown(nlohmann::json& state)
        {}
    }

}
