#ifndef PERIODIC_ACTION_HPP
#define PERIODIC_ACTION_HPP

#include <memory>

#include <action.hpp>

namespace rip
{

    namespace utilities
    {
        namespace units
        {
            class Time;
        }
    }

    namespace scheduler
    {
        class PeriodicAction: public Action
        {
            using Time = utilities::units::Time;
        public:
            PeriodicAction(std::shared_ptr<Action> action, Time period);

        protected:
            void initialize() override;
            void execute() override;
            void isFinished() override;
            void interrupted() override;
            void end() override;
        private:
            std::shared_ptr<Action> m_action;
            Time m_period;
            Time m_last_run;
        };
    }
}

#endif // PERIODIC_ACTION_HPP