#ifndef SUBSYSTEM_HPP
#define SUBSYSTEM_HPP

#include <string>
#include <memory>

namespace rip
{
    namespace scheduler
    {
        // Forward declarations allow for faster compilation
        class Action;

        /**
         * Represents a system on the robot. Each system should extend class
         * in order to work with the Scheduler. Each system has a default
         * action which runs when no other action runs.
         */
        class Subsystem
        {
            /**
             * @brief Constructor
             * @param name The name of the system
             */
            Subsystem(std::string name);

            /**
             * Gets the name of the system
             * @returns The name of the system
             */
            std::string getName();

            /**
             * Sets the name of the system
             * @param name The name of the system
             */
            void setName(std::string name);

            /**
             * Cancels the current action running on this system if there is one
             */
            void cancelCurrentAction();

            /**
             * Gets whether this system has a current action
             * @returns true if there is a current action, false otherwise
             */
            bool hasCurrentAction();

            /**
             * Sets the current action
             * @param action The action that should be run for this system
             */
            void setCurrentAction(std::shared_ptr<Action> action);

            /**
             * Gets the current action for this system
             * @returns The current action for this system
             */
            std::shared_ptr<Action> getCurrentAction();

            /**
             * Starts the default action and adds it to the scheduler
             */
            void startDefaultAction();

            /**
             * Sets the default action for this system
             * @param action The default action
             */
            void setDefaultAction(std::shared_ptr<Action> action);

        private:
            std::string m_name;

            std::shared_ptr<Action> m_default_action;
            std::shared_ptr<Action> m_current_action;
        };
    }
}

#endif // SUBSYSTEM_HPP