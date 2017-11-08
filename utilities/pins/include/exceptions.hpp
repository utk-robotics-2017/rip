#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP
#include <exception_base.hpp>

namespace rip {
    namespace pins {

        /* @class FileAccessError
         * @brief Used for when a file cannot be accessed (ex. permissions or existence)
         */
        NEW_EX(FileAccessError);

        /* @class DigitalReadError
         * @brief Used when we don't get a proper read from a digital input (ex. not 0|1)
         */
        NEW_EX(DigitalReadError);

    }
}

#endif // EXCEPTIONS_HPP
