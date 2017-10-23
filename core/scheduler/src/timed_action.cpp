#include "timed_action.hpp"

#include <time.h>

#include <units.hpp>

namespace rip
{
    namespace scheduler
    {
        TimedAction::TimedAction(std::shared_ptr<Action> action, Time time)
            : m_action(action)
            , m_time(time)
        {
            time_t  timev;
            time(&timev);

            m_start_time = timev * units::s;
        }

        void TimedAction::initialize() override
        {
            m_action->initialize();
        }

        void TimedAction::execute() override
        {
            m_action->execute();
        }

        bool TimedAction::isFinished() override
        {
            time_t  timev;
            time(&timev);
            Time current_time = timev * units::s;
            return m_action->isFinished() || current_time >= m_time + m_start_time;
        }

        void TimedAction::interrupted() override
        {
            m_action->interrupted();
        }

        void TimedAction::end() override
        {
            m_action->end();
        }
    }
}