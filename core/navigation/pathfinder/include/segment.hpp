#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct Segment
            {
                Time time;
                Distance distance;
                Velocity velocity;
                Acceleration acceleration;
            };
        }
    }
}

#endif // SEGMENT_HPP
