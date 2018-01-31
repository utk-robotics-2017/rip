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
#ifndef WAIT_ACTION_HPP
#define WAIT_ACTION_HPP

#include <chrono>
#include <ctime>

#include <units.hpp>

#include "action.hpp"

namespace rip
{
    namespace core
    {
        namespace framework
        {
            /**
             * Action to wait for a given amount of time
             */
            class WaitAction : public Action
            {
            public:
                /**
                 * Constructor
                 *
                 * @param wait_time The amount of time for this action to wait
                 */
                WaitAction(const units::Time& wait_time);

                /**
                 * Finishes when the elapsed time is more than the wait time
                 */
                virtual bool isFinished() override;

                /**
                 * [setup description]
                 */
                virtual void setup() override;

                /**
                 * [update description]
                 */
                virtual void update() override;

                /**
                 * [teardown description]
                 */
                virtual void teardown() override;

                /**
                 * Saves the elapsed time
                 */
                virtual nlohmann::json save() const override;

                /**
                 * Restores the action
                 */
                virtual void restore(const nlohmann::json& state) override;

            private:
                units::Time m_wait_time;
                std::chrono::time_point<std::chrono::system_clock> m_start_time;
            };
        }
    }
}

#endif // WAIT_ACTION_HPP
