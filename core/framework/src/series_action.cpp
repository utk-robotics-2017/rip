#include "series_action.hpp"

namespace rip
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

    void SeriesAction::setup() {}

    void SeriesAction::update()
    {
        if (m_current != m_previous)
        {
            m_actions[m_current]->setup();
            m_previous = m_current;
        }
        m_actions[m_current]->update();
        if (m_actions[m_current]->isFinished())
        {
            m_actions[m_current]->teardown();
            m_current++;
        }
    }

    void SeriesAction::teardown() {}

    nlohmann::json SeriesAction::save() const
    {
        nlohmann::json j;
        j["previous"] = m_previous;
        j["current"] =  m_current;
        j["substate"] = m_actions[m_current]->save();
        return j;
    }

    void SeriesAction::restore(const nlohmann::json& state)
    {
        m_previous = state["previous"];
        m_current = state["current"];
        m_actions[m_current]->restore(state["substate"]);
    }

}
