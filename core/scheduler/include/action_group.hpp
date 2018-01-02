#ifndef ACTION_GROUP_HPP
#define ACTION_GROUP_HPP

#include <memory>
#include <vector>
#include <pair>

#include <spdlog>

#include <action.hpp>

namespace rip
{
    namespace scheduler
    {
        class ActionGroup : public Action
        {
        public:
            /**
             * Creates an empty action group
             */
            ActionGroup();

            /**
             * Creates an action group and adds the list of actions to run sequentially
             * @param actions A list of actions to add
             */
            ActionGroup(const std::vector< std::shared_ptr<Action> >& actions);

            /**
             * Creates an action group and add the list of actions
             * @param sequential If true, the actions will run sequentially otherwise in parallel
             * @param actions A list of actions to add
             */
            ActionGroup(bool sequential, const std::vector< std::shared_ptr<Action> >& actions);

            /**
             * Adds an action to run sequentially
             * @param action The action to add
             */
            void addSequential(std::shared_ptr<Action> action);

            /**
             * Adds a list of actions to run sequentially
             * @param actions A list of actions to add
             */
            void addSequential(const std::vector< std::shared_ptr<Action> >& actions);

            /**
             * Adds an empty action that runs for time \p wait_time
             * @param wait_time The amount of time for the action to run
             */
            void addWaitAction(units::Time wait_time);

            /**
             * Adds an action to run in parallel
             * @param action The action to run
             */
            void addParallel(std::shared_ptr<Action> action);

            /**
             * Adds a list of actions to run in parallel
             * @param actions A listof actions to run
             */
            void addParallel(std::vector< std::shared_ptr<Action> > actions);

        protected:
            void initialize() override;

            void execute() override;

            bool isFinished() override;

            void end() override;

        private:
            using ActionEntry = std::pair<std::shared_ptr<Action>, bool>;
            std::vector< ActionEntry > m_actions;
            std::vector< ActionEntry > m_current;
            int m_index;

            std::shared_ptr<spdlog::logger> m_logger;
        };
    }
}

#endif ACTION_GROUP_HPP