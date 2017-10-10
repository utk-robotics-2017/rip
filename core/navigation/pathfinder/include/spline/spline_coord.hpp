#ifndef SPLINE_COORD_HPP
#define SPLINE_COORD_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct SplineCoord
            {
                Time time;
                Distance x;
                Distance y;
                Angle angle;
            };
        }
    }
} // namespace rip

#endif // SPLINE_COORD_HPP