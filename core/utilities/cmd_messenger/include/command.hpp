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

#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <string>

namespace rip
{
    namespace cmdmessenger
    {
        /**
             * @class Command
             * @brief Class containing the metadata for an individual command
             */
        class Command
        {
        public:
            /**
                 * @brief Constructor
                 *
                 * @param id The name of the command
                 * @param enum_ The enum number for this commane
                 */
            Command(const std::string& id, int enum_, const std::string& argument_types);

            /**
                 * @brief Returns the name of the command
                 *
                 * @return The name of the command
                 */
            std::string getId() const;

            /**
                 * @brief Returns the enum number for this command
                 * @return The enum number for this command
                 */
            int getEnum() const;

            /**
                 * @brief Gets the argument types for this command
                 * @return The argument types for this command
                 */
            std::string getArgumentTypes() const;

        private:
            std::string m_id;
            int m_enum;
            std::string m_argument_types;
        };
    }
}
#endif // COMMAND_HPP
