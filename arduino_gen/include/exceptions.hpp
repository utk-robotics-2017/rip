#ifndef ARDUINOGEN_EXCEPTIONS_HPP
#define ARDUINOGEN_EXCEPTIONS_HPP
#include <exception_base.hpp>

namespace rip
{
    namespace arduinogen
    {
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
}

#endif // ARDUINOGEN_EXCEPTIONS_HPP
