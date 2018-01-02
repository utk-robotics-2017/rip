#include "drivetrain.hpp"

namespace rip
{
    namespace subsystem
    {

        bool Drivetrain::diagnostic()
        {
            drive(1.0);
            drive(-1.0);

            drive(1.0, 0.0);
            drive(-1.0, 0.0);
            drive(0.0, 1.0);
            drive(0.0, -1.0);

            drive(1.0, 0.0, 0.0, 0.0);
            drive(-1.0, 0.0, 0.0, 0.0);
            drive(0.0, 1.0, 0.0, 0.0);
            drive(0.0, -1.0, 0.0, 0.0);
            drive(0.0, 0.0, 1.0, 0.0);
            drive(0.0, 0.0, -1.0, 0.0);
            drive(0.0, 0.0, 0.0, 1.0);
            drive(0.0, 0.0, 0.0, -1.0);
        }
    }
}
