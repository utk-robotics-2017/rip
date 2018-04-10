#ifndef MOTION_SEGMENT_HPP
#define MOTION_SEGMENT_HPP

#include <ostream>

#include <units/units.hpp>

#include "path_follower/motion_state.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A MotionSegment is a movement from a start MotionState to an end MotionState with a constant acceleration
             */
            class MotionSegment
            {
            public:
                MotionSegment(const MotionState& start, const MotionState& end);

                /**
                 * Verifies that:
                 *
                 * 1. All segments have a constant acceleration
                 *
                 * 2. All segments have monotonic position (sign of velocity doesn't change)
                 *
                 * 3. The time, position, velocity, and acceleration of the profile are consistent
                 */
                bool valid() const;

                bool containsTime(const units::Time& t) const;

                bool containsPosition(const units::Distance& pos) const;

                MotionState start() const;

                void setStart(const MotionState& state);

                MotionState end() const;

                void setEnd(const MotionState& state);

                std::string toString() const;

                friend std::ostream& operator<<(std::ostream& os, const MotionSegment& segment);

            private:
                MotionState m_start;
                MotionState m_end;
            };

            std::ostream& operator<<(std::ostream& os, const MotionSegment& segment);
        }
    }
}

#endif //MOTION_SEGMENT_HPP
