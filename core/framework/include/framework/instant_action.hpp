/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef RUN_ONCE_ACTION_HPP
#define RUN_ONCE_ACTION_HPP

#include "action.hpp"

namespace rip
{
    namespace framework
    {
        /**
         * Base action for something that only needs to be done
         * once
         *
         * @note Only runOnce should be overloaded for an InstantAction derivative
         */
        class InstantAction : public Action
        {
        public:
            InstantAction(const std::string& name);

            /**
             * Returns whether the action is finished
             *
             * @note Is always true
             */
            bool isFinished() final override;

            /**
             * Action has already happened, so nothing
             */
            void update(nlohmann::json& state) final override;

            /**
             * Where the action actually does stuff
             */
            void setup(nlohmann::json& state) final override;

            /**
             * Never happens
             */
            void teardown(nlohmann::json& state) final override;

            /**
             * This is where the magic happens for this action
             */
            virtual void runOnce(nlohmann::json& state) = 0;
        };
    }
}

#endif // RUN_ONCE_ACTION_HPP
