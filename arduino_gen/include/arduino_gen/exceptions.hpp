#ifndef ARDUINO_GEN_EXCEPTIONS_HPP
#define ARDUINO_GEN_EXCEPTIONS_HPP
#include <string>
#include <misc/exception_base.hpp>

namespace rip
{
    namespace arduinogen
    {
        /**
         * @class AppendageDataException
         * @brief An exception for when there is an issue with data in an appendage
         */
        NEW_EX(AppendageDataException)

        /**
         * @class AttributeException
         * @brief An exception for when there is an xml attribute issue
         */
        NEW_EX(AttributeException)

        /**
         * @class ElementException
         * @brief An exception for when there is an xml element issue
         */
        NEW_EX(ElementException)

        /**
         * @class TypeException
         * @brief An exception for when there is a type error
         */
        NEW_EX(TypeException)

        /**
         * @class FileIoException
         * @brief An exception for when there is an error opening, reading, or writing a file.
         */
        NEW_EX(FileIoException)

        /**
         * @class PatternNotFoundException
         * @brief An exception for when a regex pattern was not found.
         */
        NEW_EX(PatternNotFoundException)
    }
}

#endif // ARDUINO_GEN_EXCEPTIONS_HPP
