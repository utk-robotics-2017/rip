#ifndef COUPLED_SEGMENT_HPP
#define COUPLED_SEGMENT_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct CoupledSegment
            {
                std::shared_ptr<Segment> center;
                std::shared_ptr<Segment> left;
                std::shared_ptr<Segment> right;

                std::shared_ptr<Segment2D> center_2d;
                std::shared_ptr<Segment2D> left_2d;
                std::shared_ptr<Segment2D> right_2d;
            };
        }
    }
}

#endif // COUPLED_SEGMENT_HPP