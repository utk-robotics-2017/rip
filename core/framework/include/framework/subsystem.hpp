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
#ifndef SUBSYSTEM_HPP
#define SUBSYSTEM_HPP

#include <string>
#include <ostream>
#include <istream>

namespace rip
{
    namespace framework
    {
        class Subsystem
        {
        public:
            Subsystem(const std::string& name);

            virtual ~Subsystem();

            void setName(const std::string& name);

            std::string name() const;

            virtual bool diagnostic() = 0;
            virtual void stop() = 0;

            /*
                        void setDiagOut(std::ostream output)
                        {
                            m_diag_output = output;
                        }

                        void setDiagIn(std::istream input)
                        {
                            m_diag_input = input;
                        }
                        */
        private:
            std::string m_name;
        };
    }
}

#endif // SUBSYSTEM_HPP
