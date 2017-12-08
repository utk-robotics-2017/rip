#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

namespace rip
{
    namespace scheduler
    {
        /**
         * @class Scheduler
         * @brief The Scheduler is responsible for executing actions for robot systems
         *
         * Scheduler contains a collection of actions, systems and scheduled tasks. When
         * a system is created it is added automatically by its constructor. Actions are
         * added when they are started. ScheduledTasks are added manually. To avoid
         * collisions between actions running on the same system, when an action is added
         * its system requirements are checked and any actions using those systems are canceled.
         * If registered systems are not given an action to run but do have a default action,
         * the default action is started.
         */
        class Scheduler
        {
        public:
            static std::shared_ptr<Scheduler> getInstance();

            /**
             * Runs the scheduler
             */
            void run();

            /**
             * Adds a subsystem
             *
             * @param system The subsystem to add
             */
            void registerSystem(std::shared_ptr<Subsystem> system);

            /**
             * Adds a new {@link Action} to be executed by the scheduler.
             *
             * The action's system requirements are check. If other actions use those same
             * systems, those actions are cancelled. If the scheduler is disabled, the action
             * cannot be added.
             *
             * @param  action The action to acdd
             * @returns true if the action was successfully added
             */
            bool add(std::shared_ptr<Action> action);

            /**
             * Removes an {@link Action} from the scheduler.
             *
             * If the action is in the scheduler, it is removed and its requirements are updated
             * as lacking a current action.
             *
             * @param action The action to remove
             */
            void remove(std::shared_ptr<Action> action);

        private:
            Scheduler();

            static std::shared_ptr<Scheduler> m_singleton;

            bool m_disabled;
            std::vector< std::shared_ptr<Action> > m_actions;
            std::vector< std::shared_ptr<Subsystem> > m_subsystems;
        };
    }
} // rip

#endif // SCHEDULER_HPP