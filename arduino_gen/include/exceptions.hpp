#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP
#include <string>

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
     * @class AppendageDataException
     * @brief An exception for when there is an issue with data in an appendage
     */
    NEW_EX(AppendageDataException);

    /**
     * @class AttributeException
     * @brief An exception for when there is an xml attribute issue in the TemplateParser
     */
    NEW_EX(AttributeException);

    /**
     * @class TypeException
     * @brief An exception for when there is a type error
     */
    NEW_EX(TypeException);
}

#endif // EXCEPTIONS_HPP
