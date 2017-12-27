#include "spline.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
        {
            Distance Spline::totalLength()
            {
                Distance result = 0;
                for (size_t i = 0, end = numSegments(); i < end; i++)
                {
                    result += segmentArcLength(i);
                }
                return result;
            }
        }
    }
}
