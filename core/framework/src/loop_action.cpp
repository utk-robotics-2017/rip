#include <misc/logger.hpp>
#include "framework/loop_action.hpp"

namespace rip
{
    namespace framework
    {
        LoopAction::LoopAction(const std::string& name, std::shared_ptr <Action> action, std::shared_ptr <Action> reset_action, std::shared_ptr <Condition> condition)
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
            rip::misc::Logger::getInstance()->debug("R: {} PR: {}", m_reset, m_previous_reset);
            if (m_reset)
            {

                if (!m_previous_reset)
                {
                    rip::misc::Logger::getInstance()->debug("Reset Action Setup");
                    m_reset_action->setup(state);
                }
                else
                {
                    if (m_reset_action->isFinished())
                    {
                        rip::misc::Logger::getInstance()->debug("Reset Action Finished");
                        m_reset_action->teardown(state);
                        m_reset = false;
                    }
                    else
                    {
                        rip::misc::Logger::getInstance()->debug("Reset Action Update");
                        m_reset_action->update(state);
                    }
                }

                m_previous_reset = true;
            }
            else
            {
                if (m_previous_reset)
                {
                    rip::misc::Logger::getInstance()->debug("Action Setup");
                    m_action->setup(state);
                }
                else
                {
                    if (m_action->isFinished())
                    {
                        rip::misc::Logger::getInstance()->debug("Action Finished");
                        m_action->teardown(state);
                        if (m_condition->isTrue())
                        {
                            rip::misc::Logger::getInstance()->debug("Resetting");
                            m_reset = true;
                        }
                        else
                        {
                            m_finished = true;
                        }
                    }
                    else
                    {
                        rip::misc::Logger::getInstance()->debug("Action Update");
                        m_action->update(state);
                    }
                }

                m_previous_reset = false;
            }
        }

        void LoopAction::teardown(nlohmann::json& state)
        {
            rip::misc::Logger::getInstance()->debug("Loop Teardown");
        }

        bool LoopAction::isFinished()
        {
            return m_finished;
        }
    }
}