#ifndef DRIVETRAINS_EXCEPTIONS_HPP
#define DRIVETRAINS_EXCEPTIONS_HPP

#include <misc/exception_base.hpp>

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            /**
             * Provided value is out of range
             * @param OutOfRangeException [description]
             */
            NEW_EX(OutOfRangeException)
            /**
             * attempt to use a motor that does not exist,
             * ie requesting encoder informoation from back right motor in a 2 wheel configuration
             * @param InvalidEncoderRequest [description]
             */
            NEW_EX(InvalidMotorException)

        }
    }
}

#endif // DRIVETRAINS_EXCEPTIONS_HPP
