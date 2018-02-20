#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP
#include <misc/exception_base.hpp>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            /**
             * @class UnknownDistanceSettingException
             */
            NEW_EX(UnknownDistanceSettingException);

            /**
             * @class UnknownTimeSettingException
             */
            NEW_EX(UnknownTimeSettingException);

            /**
             * @class UnknownVelocitySettingException
             */
            NEW_EX(UnknownVelocitySettingException);

            /**
             * @class UnknownAccelerationSettingException
             */
            NEW_EX(UnknownAccelerationSettingException);

            /**
             * @class UnknownAngleSettingException
             */
            NEW_EX(UnknownAngleSettingException);

            /**
             * @class UnknownAngularVelocitySettingException
             */
            NEW_EX(UnknownAngularVelocitySettingException);

            /**
             * @class UnknownAngularAccelerationSettingException
             */
            NEW_EX(UnknownAngularAccelerationSettingException);

            /**
             * @class UnknownDoubleSettingException
             */
            NEW_EX(UnknownDoubleSettingException);

            /**
             * @class UnknownIntegerSettingException
             */
            NEW_EX(UnknownIntegerSettingException);

            /**
             * @class UnknownBooleanSettingException
             */
            NEW_EX(UnknownBooleanSettingException);

            /**
             * @class UnknownObstacle
             */
            NEW_EX(UnknownObstacle);
        }
    }
}

#endif // EXCEPTIONS_HPP
