#include "path_follower/motion_segment.hpp"
#include "path_follower/epsilon.hpp"

#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionSegment::MotionSegment(const MotionState& start, const MotionState& end)
                    : m_start(start)
                    , m_end(end)
            {
            }

            bool MotionSegment::valid() const
            {
                if(!epsilonEquals<units::Acceleration>(m_start.acceleration(), m_end.acceleration(), k_epsilon))
                {
                    misc::Logger::getInstance()->error("Segment acceleration not constant! Start acc: {} in/s^2, End acc: {} in/s^2",
                                                       m_start.acceleration().to(units::in / units::s / units::s),
                                                       m_end.acceleration().to(units::in / units::s / units::s));
                    return false;
                }

                if(units::signum(m_start.velocity()) * units::signum(m_end.velocity()) < 0 &&
                        !epsilonEquals<units::Velocity>(m_start.velocity(), 0.0, k_epsilon) &&
                        !epsilonEquals<units::Velocity>(m_end.velocity(), 0.0, k_epsilon))
                {
                    misc::Logger::getInstance()->error("Segment velocity reverses! Start vel: {} in/s, End vel: {} in/s",
                                                       m_start.velocity().to(units::in / units::s),
                                                       m_end.velocity().to(units::in / units::s));
                    return false;
                }

                if(m_start.extrapolate(m_end.t()) != m_end)
                {
                    // A single segment is not consistent
                    if(m_start.t() == m_end.t() && units::isinf(m_start.acceleration()))
                    {
                        // One allowed exception: If acceleration is infinite and dt is zero
                        return true;
                    }
                    misc::Logger::getInstance()->error("Segment not consistent! Start: {}, End: {}", m_start.toString(), m_end.toString());
                    return false;
                }

                return true;
            }

            bool MotionSegment::containsTime(const units::Time& t) const
            {
                return t >= m_start.t() && t <= m_end.t();
            }

            bool MotionSegment::containsPosition(const units::Distance& pos) const
            {
                return pos >= m_start.position() && pos <= m_end.position() || pos <= m_start.position() && pos >= m_end.position();
            }

            MotionState MotionSegment::start() const
            {
                return m_start;
            }

            void MotionSegment::setStart(const MotionState& state)
            {
                m_start = state;
            }

            MotionState MotionSegment::end() const
            {
                return m_end;
            }

            void MotionSegment::setEnd(const MotionState& state)
            {
                m_end = state;
            }

            std::string MotionSegment::toString() const
            {
                return fmt::format("Start: {}, End: {}", m_start.toString(), m_end.toString());
            }

            std::ostream& operator<<(std::ostream& os, const MotionSegment& segment)
            {
                os << segment.toString() << std::endl;
            }
        }
    }
}