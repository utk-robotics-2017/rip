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
#ifndef ACTION_HPP
#define ACTION_HPP

#include <json.hpp>

namespace rip
{
    /**
     * Abstract Action Base class
     */
    class Action
    {
    public:
        /**
         * Returns whether or not the action has finished execution.
         */
        virtual bool isFinished() = 0;

        /**
         * Iteratively called until {@see Action#isFinished()} returns true
         */
        virtual void update() = 0;

        /**
         * Run once before the main code
         */
        virtual void setup() = 0;

        /**
         * Run once after finished
         */
        virtual void teardown() = 0;

        /**
         * Returns the state which can be used in case of a crash
         */
        virtual nlohmann::json save() const = 0;

        /**
         * Restores the action to state
         */
        virtual void restore(const nlohmann::json& state) = 0;
    };
}

#endif // ACTION_HPP
