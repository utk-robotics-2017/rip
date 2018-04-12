#include "framework/series_action.hpp"

namespace rip
{
    namespace framework
    {
        SeriesAction::SeriesAction(const std::string& name, const std::vector<std::shared_ptr<Action> >& actions)
            : Action(name)
            , m_actions(actions)
            , m_current(0)
            , m_previous(-1)
        {}

        bool SeriesAction::isFinished()
        {
            /**
             * We're on the last action, and that action is finished.
             */
            return
              m_current == static_cast<int>(m_actions.size())
              && m_actions[m_current]->isFinished();
        }

        void SeriesAction::setup(nlohmann::json& state) {}

        void SeriesAction::update(nlohmann::json& state)
        {
            // don't run if we're past the end
            if (m_current >= m_actions.size()) return;

            // If new sub-action then set it up
            if (m_current != m_previous)
            {
                m_actions[m_current]->setup(state);
                m_previous = m_current;
            }

            // Update
            m_actions[m_current]->update(state);

            // If action is done then tear it down
            if (m_actions[m_current]->isFinished())
            {
                m_actions[m_current]->teardown(state);
                m_current++;
            }
        }

        void SeriesAction::teardown(nlohmann::json& state) {}
    }
}
