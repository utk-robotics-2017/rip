#include "framework/loop_action.hpp"

namespace rip
{
    namespace framework
    {
        LoopAction::LoopAction(const std::string& name, std::shared_ptr <Action> action, std::shared_ptr <Action> reset_action, std::shared_ptr <LoopCondition> condition)
            : Action(name)
            , m_action(action)
            , m_reset_action(reset_action)
            , m_condition(condition)
            , m_reset(false)
            , m_previous_reset(true)
        {
        }

        void LoopAction::setup(nlohmann::json& state)
        {

        }

        void LoopAction::update(nlohmann::json& state)
        {
            if(m_reset)
            {
                if(m_reset_action->isFinished())
                {
                    m_reset_action->teardown(state);
                    m_reset = false;
                }
                else
                {
                    if(m_previous_reset)
                    {
                        m_reset_action->update(state);
                    }
                    else
                    {
                        m_reset_action->setup(state);
                    }
                }
                m_previous_reset = true;
            }
            else
            {
                if(m_action->isFinished())
                {
                    m_action->teardown(state);
                    if(m_condition->loop())
                    {
                        m_reset = true;
                    }
                    else
                    {
                        m_finished = true;
                    }
                }
                else
                {
                    if(m_previous_reset)
                    {
                        m_action->setup(state);
                    }
                    else
                    {
                        m_action->update(state);
                    }
                }
                m_previous_reset = false;
            }
        }

        void LoopAction::teardown(nlohmann::json& state)
        {

        }

        bool LoopAction::isFinished()
        {
            return m_finished;
        }
    }
}