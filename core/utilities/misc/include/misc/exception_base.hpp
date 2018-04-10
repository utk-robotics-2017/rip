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
#include <utility>
#include <fmt/format.h>

#define NEW_EX(name) \
class name : public rip::utilities::ExceptionBase \
{ \
public: \
    template <typename... Args>\
    name(const std::string& message="", Args && ... args) \
        : rip::utilities::ExceptionBase(#name ": " + message, std::forward<Args>(args)...)\
    {} \
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
            /**
             *  Constructor with parameter pack template and perfect forwarding
             */
            template <typename... Args>
            ExceptionBase(const std::string& message,  Args && ... args)
                : m_message(fmt::format(message, std::forward<Args>(args)...))
            {}

            const char* what() const throw()
            {
                return m_message.c_str();
            }
        };
    }
}
#endif // EXCEPTION_BASE_HPP
