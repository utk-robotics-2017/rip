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
#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP
#include <string>

namespace cmdmessenger
{
    class Exception : public std::exception
    {
    protected:
        std::string m_message;

    public:
        Exception(std::string message = "") : m_message(message) {}
        const char* what() const throw()
        {
            return m_message.c_str();
        }
    };

    class EmptyDevice : public Exception
    {
    public:
        EmptyDevice(std::string message = "") : Exception(message) {}
    };

    class EmptyCommand : public Exception
    {
    public:
        EmptyCommand(std::string message = "") : Exception(message) {}
    };

    class IncorrectAcknowledgementCommand : public Exception
    {
    public:
        IncorrectAcknowledgementCommand(std::string message = "") : Exception(message) {}
    };

    class IncorrectArgumentListSize : public Exception
    {
    public:
        IncorrectArgumentListSize(std::string message = "") : Exception(message) {}
    };

    class IncorrectCommandSeparator : public Exception
    {
    public:
        IncorrectCommandSeparator(std::string message = "") : Exception(message) {}
    };

    class IncorrectFieldSeparator : public Exception
    {
    public:
        IncorrectFieldSeparator(std::string message = "") : Exception(message) {}
    };

    class IncorrectResponseCommand : public Exception
    {
    public:
        IncorrectResponseCommand(std::string message = "") : Exception(message) {}
    };

    class OutOfBounds : public Exception
    {
    public:
        OutOfBounds(std::string message = "") : Exception(message) {}
    };

    class NoLastDeviceException : public Exception
    {
    public:
        NoLastDeviceException(std::string message = "") : Exception(message) {}
    };

    class UnconvertibleArgument : public Exception
    {
    public:
        UnconvertibleArgument(std::string message = "") : Exception(message) {}
    };

    class UnknownArgument : public Exception
    {
    public:
        UnknownArgument(std::string message = "") : Exception(message) {}
    };
}
#endif // EXCEPTIONS_HPP
