#include "subsystem.hpp"

#include "action.hpp"

namespace rip
{
    namespace scheduler
    {
        Subsystem::Subsystem(std::string name)
            : m_name(name)
        {
            Scheduler.getInstance().registerSystem(this);
        }

        std::string Subsystem::getName()
        {
            return m_name;
        }

        void Subsystem::setName(std::string name)
        {
            m_name = name;
        }

        void Subsystem::cancelCurrentAction()
        {
            if (hasCurrentAction() && m_current_action->isRunning())
            {
                m_current_action->cancel();
            }
        }

        bool Subsystem::hasCurrentAction()
        {
            return m_current_action; //!< Implicitly converts to boolean based on if it is not a nullptr
        }

        void Subsystem::setCurrentAction(std::shared_ptr<Action> action)
        {
            m_current_action = action;
        }

        std::shared_ptr<Action> Subsystem::getCurrentAction()
        {
            return m_current_action;
        }

        void Subsystem::startDefaultAction()
        {
            if (m_default_action)
            {
                m_default_action->start();
            }
        }

        void Subsystem::setDefaultAction(std::shared_ptr<Action> action)
        {
            m_default_action = action;
        }
    }
}