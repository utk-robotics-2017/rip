#include "action_group.hpp"

#include "timed_action.hpp"

namespace rip
{
    namespace scheduler
    {
        ActionGroup::ActionGroup()
            : Action()
            , m_logger(spdlog::get("rip_logger"))
            , m_index(-1)
        {}

        ActionGroup::ActionGroup(const std::vector< std::shared_ptr<Action> >& actions)
        {
            addSequential(std::forward< std::vector< std::shared_ptr<Action> > >(actions));
        }

        ActionGroup::ActionGroup(bool sequential, const std::std::vector< std::shared_ptr<Action> >& actions)
        {
            if (sequential)
            {
                addSequential(std::forward< std::vector< std::shared_ptr<Action> > >(actions));
            }
            else
            {
                addParallel(std::forward< std::vector< std::shared_ptr<Action> > >(actions));
            }
        }

        void ActionGroup::addSequential(std::shared_ptr<Action> action)
        {
            m_actions.push_back(std::make_pair<std::shared_ptr<Action>, bool>(action, true));
        }

        void ActionGroup::addSequential(const std::vector< std::shared_ptr<Action> >& actions)
        {
            for (std::shared_ptr<Action> action : actions)
            {
                addSequential(action);
            }
        }

        void ActionGroup::addWaitAction(units::Time wait_time)
        {
            addSequential(std::make_shared<TimedAction>(Action.kEmpty, wait_time));
        }

        void ActionGroup::addParallel(std::shared_ptr<Action> action)
        {
            m_actions.push_back(std::make_pair<std::shared_ptr<Action>, bool>()action, false);
        }

        void ActionGroup::addParallel(std::vector< std::shared_ptr< Action> > actions)
        {
            for (std::shared_ptr<Action> action : actions)
            {
                addParallel(action);
            }
        }

        void ActionGroup::initialize() override
        {
            m_index = 0;
            ActionEntry c = m_actions[m_index];
            m_logger->info("Starting action-{}", c.first->getName());
            c.first->start();
            m_current.push_back(c);
        }

        void ActionGroup::execute() override
        {
            ActionEntry c = m_actions[m_index];
            bool next = false;
            if (!c.first->isRunning())
            {
                m_current.remove(c);
                m_index++;
                next = true;
            }
            else if (!c.second)
            {
                m_index++;
                next = true;
            }

            for (ActionEntry entry : m_current)
            {
                if (!entry.first->isRunning())
                {
                    m_current.remove(entry.first);
                }
            }

            if (next && m_index < m_actions.size())
            {
                ActionEntry to_run = m_actions[m_index];
                m_logger->info("Starting action-{}", to_run.first->getName());
                to_run.first->start();
                m_current.push_back(to_run);
            }
        }

        bool ActionGroup::isFinished() override
        {
            return m_index >= m_actions.size();
        }

        void ActionGroup::end() override
        {
            for (ActionEntry entry : m_current)
            {
                if (entry.first->isRunning())
                {
                    entry.first->cancel();
                }
            }

            m_current.clear();
            m_index = -1;
        }
    }
}