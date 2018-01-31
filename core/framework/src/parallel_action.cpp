#include "parallel_action.hpp"

namespace rip
{
    namespace core
    {
        namespace framework
        {
            ParallelAction::ParallelAction(const std::vector<std::shared_ptr<Action> >& actions)
                : m_actions(actions)
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

            void ParallelAction::update()
            {
                for (std::shared_ptr<Action> action : m_actions)
                {
                    action->update();
                }
            }

            void ParallelAction::setup()
            {
                for (std::shared_ptr<Action> action : m_actions)
                {
                    action->setup();
                }
            }

            void ParallelAction::teardown()
            {
                for (std::shared_ptr<Action> action : m_actions)
                {
                    action->teardown();
                }
            }

            nlohmann::json ParallelAction::save() const
            {
                nlohmann::json j;
                for (std::shared_ptr<Action> action : m_actions)
                {
                    j.push_back(action->save());
                }
                return j;
            }

            void ParallelAction::restore(const nlohmann::json& state)
            {
                for (int i = 0, end = m_actions.size(); i < end; i++)
                {
                    m_actions[i]->restore(state[i]);
                }
            }
        }
    }
}
