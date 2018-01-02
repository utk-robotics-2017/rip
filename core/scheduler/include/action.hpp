#ifndef ACTION_HPP
#define ACTION_HPP

#include <string>
#include <memory>
#include <vector>

namespace rip
{
    namespace scheduler
    {
        /**
         * @brief An Action is something that can be executed by a
         *        System on the robot.
         *
         * 	When started, the action is added to the Scheduler which
         * 	is responsible for running it. The Action is ran until a
         * 	condition is met or an action using the same system is used.
         *
         *  <p>
         *  An action has a few running phases:
         *  <ul>
         *  	<li> {@link #initialize()} called once at the start </li>
         *  	<li> {@link #execute()} called repeatedly until
         *  	{@link #isFinshed()} returns true </li>
         *  	<li> {@link #end()} called when the action is done
         *  	(when {@link isFinished()} returns true) </li>
         *  	<li> {@link #interruptted()} called when another another
         *  	starts running on the same system, or the action is manually cancelled</li>
         *  </ul>
         *
         * <p>
         * There are many different types of wrappers for action. The wrappers allow for
         * the modification of an action without changing the defining parameters
         * <ul>
         * 		<li> {@link TimedAction} wraps an action and sets a timeout for it</li>
         * 		<li> {@link ActionGroup} a series of actions to be executed in a specific order</li>
         * 		<li> {@link ConditionalAction} selects which action to run based on a condition</li>
         * 		<li> {@link InstantAction} An action in which {@link #execute()} only happens once</li>
         * </ul>
         */
        class Action
        {
        public:
            /**
             * Default Constructor
             */
            Action() = default;

            /**
             * Constructor
             *
             * @param The name of the action
             */
            Action(std::string name);

            /**
             * Starts the action. If the Scheduler was initialized, then the action is added
             * to the Scheduler for running. If the action is already running then it is not added.
             */
            void start();

            /**
             * Cancels the operation of the action if it is running
             */
            void cancel();

            /**
             * Gets the name of the action
             * @return The name of the action
             */
            std::string getName();

            /**
             * Sets the name of the action
             * @param name The name of the action
             */
            void setName(std::string name);

            /**
             * Gets whether or not an action has been canceled.
             * @returns true if the action has been canceled, false otherwise
             *
             * @note If canceled that means then it did not reach {@link #end()}
             */
            bool isCancelled();


            /**
             * Gets whether or not an action is running
             * @returns true if the action is running, false otherwise
             */
            bool isRunning();

            std::vector<Subsystem> getRequirements();

        protected:
            /**
             * Adds required subsystem
             * @param subsystem [description]
             */
            void requires(std::shared_ptr<Subsystem> subsystem);

            /**
             * Adds list of required subsystems
             * @param subsystems [description]
             */
            void requires(std::vector< std::shared_ptr<Subsystem> > subsystems);

            /**
             * Gets whether or not the Scheduler should remove this action when the robot is disabled.
             *
             * @note Defaults to true
             * @return true if this action should be removed when the robot is disabled, false otherwise
             */
            virtual bool removeOnDisabled();

            /**
             * Pure virtual function for the initialization of the action
             */
            virtual void initialize() = 0;

            /**
             * Called repeatedly during execution of the action.
             */
            virtual void execute() = 0;


            /**
             * Returns true when the action is finished
             * @return [description]
             */
            virtual bool isFinished() = 0;

            /**
             * Called when the action is ended before {@link isFinished()} is true
             */
            virtual void interrupted();

            /**
             * Called when {@link #isFinished()} returns true
             */
            virtual void end() = 0;
        private:
            void removed();

            bool run();

            std::vector<Subsystem> m_subsystems;
            bool m_initialized;
            bool m_cancelled;
            bool m_running;
            std::string m_name;
        };
    }
}

#endif // ACTION_HPP