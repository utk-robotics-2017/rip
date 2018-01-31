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
#ifndef EXCEPTION_BASE_HPP
#define EXCEPTION_BASE_HPP
#include <string>

#define NEW_EX(name) \
class name : public rip::utilities::ExceptionBase \
{ \
public: \
    name(std::string message = "") : rip::utilities::ExceptionBase(#name ": " + message) {} \
};

namespace rip
{
    namespace utilities
    {
        class ExceptionBase : public std::exception
        {
        protected:
            std::string m_message;

        public:
            ExceptionBase(std::string message = "") : m_message(message) {}
            const char* what() const throw()
            {
                return m_message.c_str();
            }
        };
    }
}
#endif // EXCEPTION_BASE_HPP
