#ifndef SEGMENT2D_HPP
#define SEGMENT2D_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct Segment2D
            {
                Distance x;
                Distance y;
                Angle angle;
            };
        }
    }
}

#endif // SEGMENT2D_HPP