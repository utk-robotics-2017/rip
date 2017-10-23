#ifndef TIMED_ACTION_HPP
#define TIMED_ACTION_HPP

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
        /**
         * @class TimedAction
         * @brief An action wrapper that runs the action for a specified time
         */
        class TimedAction: public Action
        {
            using Time = utilities::units::Time;
        public:
            /**
             * Constructor
             */
            TimedAction(std::shared_ptr<Action> action, Time time);

        protected:
            void execute() override;

            bool isFinished() override;

            void interrupted() override;

            void end() override;

        private:
            std::shared_ptr<Action> m_action;
            Time m_start_time;
            Time m_time;
        };
    }
}

#endif // TIMED_ACTION_HPP