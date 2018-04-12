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
#ifndef TIMEOUT_ACTION_HPP
#define TIMEOUT_ACTION_HPP

#include <json.hpp>
#include "framework/action.hpp"
#include <units/units.hpp>

namespace rip
{
    namespace framework
    {
        /**
         * Abstract Action Base class
         */
        class TimeoutAction : public Action
        {
        public:
            TimeoutAction(const std::string& name, const nlohmann::json& config);

            /**
             * Returns whether or not the action has finished execution.
             */
            virtual bool isFinished();

            /**
             * Iteratively called until {@see Action#isFinished()} returns true
             */
            virtual void update(nlohmann::json& state);

            /**
             * Run once before the main code
             */
            virtual void setup(nlohmann::json& state);

            /**
             * Run once after finished
             */
            virtual void teardown(nlohmann::json& state);
        protected:
            bool m_has_timeout;
            units::Time m_timeout;
            units::Time m_start_time;
        };
    }
}

#endif // ACTION_HPP
