#include "pathfinder/profile/profile.hpp"
#include "pathfinder/segment.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            std::shared_ptr<Segment> Profile::calculate(std::shared_ptr<Segment> previous, const Time& time)
            {
                std::shared_ptr<Segment> temporary = std::make_shared<Segment>();
                temporary->time = previous->time;
                temporary->distance = previous->distance;
                temporary->velocity = previous->velocity;
                temporary->acceleration = previous->acceleration;

                Time dt = time - previous->time;
                int slice_count = (dt / m_timescale)();

                /*
                    The time difference provided is smaller than the target
                    timescale, use the smaller of the two
                 */
                if (slice_count < 1)
                {
                    Status status;
                    return calculateSingle(temporary, time, status);
                }
                else
                {
                    Status status;
                    for (int i = 0; i < slice_count; i++)
                    {
                        Time time_slice = temporary->time + m_timescale;
                        temporary = calculateSingle(temporary, time_slice, status);
                        if (status == Status::kDone)
                        {
                            return temporary;
                        }
                    }
                    return temporary;
                }
            }
        }
    }
}
