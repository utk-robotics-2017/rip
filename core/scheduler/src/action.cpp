#include "action.hpp"

namespace rip
{
    namespace scheduler
    {

        Action::Action(std::string name)
            : m_name(name)
        {}

        void Action::start()
        {
            // todo(Andrew): shared pointer
            if (!m_running && Scheduler.getInstance().add(this))
            {
                m_initialized = false;
                m_canceled = false;
                m_running = true;
            }
        }

        void Action::cancel()
        {
            if (m_running)
            {
                m_canceled = true;
            }
        }

        void Action::removed()
        {
            if (m_initialized)
            {
                if (m_canceled)
                {
                    interrupt();
                }
                else
                {
                    end();
                }
            }
            m_initialized = false;
            m_canceled = false;
            m_running = false;
        }

        bool Action::run()
        {
            if (m_canceled)
            {
                return false;
            }
            if (!m_initialized)
            {
                m_initialized = true;
                initialize();
            }
            execute();
            return !isFinished();
        }

        std::string Action::getName()
        {
            return m_name;
        }

        void Action::setName(std::string name)
        {
            m_name = name;
        }

        bool Action::isCancelled()
        {
            return m_cancelled;
        }

        bool Action::isRunning()
        {
            return m_running;
        }

        void Action::requires(std::shared_ptr<Subsystem> subsystem)
        {
            m_subsystems.push_back(subsystem)
        }

        void Action::requires(const std::vector< std::shared_ptr<Subsystem> >& subsystems)
        {
            m_subsystems += subsystems;
        }

        bool Action::removeOnDisabled()
        {
            return true;
        }

        void Action::interruptted()
        {
            end();
        }
    }
}