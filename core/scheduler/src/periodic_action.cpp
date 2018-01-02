#include "periodic_action.hpp"

namespace rip
{
    namespace scheduler
    {
        PeriodicAction::PeriodicAction(std::shared_ptr<Action> action, Time period)
            : m_action(action)
            , m_period(period)
        {}

        void PeriodicAction::initialize() override
        {
            m_action->initialize();
        }

        void PeriodicAction::execute() override
        {
            time_t  timev;
            time(&timev);
            Time current_time = timev * units::s;
            if (current_time >= m_last_run + m_period || m_last_run() == 0.0)
            {
                m_action->execute();
                m_last_run = current_time;
            }
        }

        bool PeriodicAction::isFinished() override
        {
            return m_action->isFinished();
        }

        void PeriodicAction::interrupted() override
        {
            m_action->interrupted();
        }

        void PeriodicAction::end() override
        {
            m_action->end();
        }
    }
}