#include "framework/series_action.hpp"

namespace rip
{
    namespace framework
    {
        SeriesAction::SeriesAction(const std::vector<std::shared_ptr<Action> >& actions)
            : m_actions(actions)
            , m_current(0)
            , m_previous(-1)
        {}

        bool SeriesAction::isFinished()
        {
            return m_current == m_actions.size();
        }

        void SeriesAction::setup(nlohmann::json& state) {}

        void SeriesAction::update(nlohmann::json& state)
        {
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
