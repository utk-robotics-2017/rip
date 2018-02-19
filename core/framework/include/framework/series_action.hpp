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
#ifndef SERIES_ACTION
#define SERIES_ACTION

#include <vector>
#include <memory>

#include "action.hpp"

namespace rip
{
    namespace framework
    {
        /**
         * Executes one action at a time.
         *
         * @note Useful as a member of {@link ParallelAction}
         */
        class SeriesAction : public Action
        {
        public:
            /**
             * Constructor
             *
             * @param action A list of actions to complete in sequential order
             */
            SeriesAction(const std::vector< std::shared_ptr<Action> >& actions);

            /**
             * Returns whether this action is finished
             */
            virtual bool isFinished() override;

            /**
             * Sets up the action
             */
            virtual void setup(nlohmann::json& state) override;

            /**
             * Updates the action
             */
            virtual void update(nlohmann::json& state) override;

            /**
             * Tears down the action
             */
            virtual void teardown(nlohmann::json& state) override;

        private:
            std::vector< std::shared_ptr<Action> > m_actions;
            int m_current;
            int m_previous;
        };
    }
}

#endif // SERIES_ACTION
