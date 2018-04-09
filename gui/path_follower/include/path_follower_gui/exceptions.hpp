#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP

#include <misc/exception_base.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                NEW_EX(RobotConfigException)
                NEW_EX(WorldConfigException)
                NEW_EX(FileNotFound)
                NEW_EX(SingularTransformException)
                NEW_EX(SelectedObstacleMissing)
                NEW_EX(UnknownSelectedType)
            }
        }
    }
}

#endif // EXCEPTIONS_HPP
