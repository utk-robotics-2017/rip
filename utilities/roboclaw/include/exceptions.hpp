#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP

#define NEW_EX(name) \
class name : public Exception \
{ \
public: \
    name(std::string message = "") : Exception(#name ": " + message) {} \
}

namespace roboclaw
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

    /**
     * @class CommandFailure
     * @brief An exception for when the command fails
     */
    NEW_EX(CommandFailure);

    /**
     * @class ReadFailure
     * @brief An exception for when the response for a read command fails
     */
    NEW_EX(ReadFailure);

    /**
     * @class OutOfRange
     * @brief An exception for when something falls out of range
     */
    NEW_EX(OutOfRange);

    /**
     * @class UnknownDType
     * @brief Unknown combination of motor dynamic parameters
     */
    NEW_EX(UnknownDType);
}

#endif //EXCEPTIONS_HPP
