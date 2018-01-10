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
#ifndef PARALLEL_ACTION_HPP
#define PARALLEL_ACTION_HPP

#include <memory>
#include <vector>

#include "action.hpp"

namespace rip
{
    /**
     * Composite action
     */
    class ParallelAction : public Action
    {
    public:
        /**
         * Constructor
         *
         * @param actions A list of actions to be completed at the same time
         */
        ParallelAction(const std::vector< std::shared_ptr<Action> >& actions);

        /**
         * Returns whether or not this action is finished
         *
         * @note Finishes when ALL internal actions are finished
         */
        virtual bool isFinished() override;

        /**
         * Updates all the internal actions
         */
        virtual void update() override;

        /**
         * Sets up all the internal actions
         */
        virtual void setup() override;

        /**
         * Tears down all the internal actions
         */
        virtual void teardown() override;

        /**
         * Returns the saved state of each of the internal actions
         * @return [description]
         */
        virtual nlohmann::json save() const override;

        /**
         * Restores each of the internal actions
         */
        virtual void restore(const nlohmann::json& state) override;

    private:
        std::vector< std::shared_ptr<Action> > m_actions;
    };
}

#endif // PARALLEL_ACTION_HPP
