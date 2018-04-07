#include "path_follower/line.hpp"
#include "path_follower/epsilon.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Line::Line(const Waypoint& a, const Waypoint& b)
                : m_a(a)
                , m_b(b)
            {}

            Waypoint Line::a() const
            {
                return m_a;
            }

            Waypoint Line::b() const
            {
                return m_b;
            }

            Translation2d Line::start() const
            {
                return m_start;
            }

            Translation2d Line::end() const
            {
                return m_end;
            }

            Translation2d Line::slope() const
            {
                return m_slope;
            }

            units::Velocity Line::speed() const
            {
                return m_speed;
            }

            void Line::addToPath(Path& p, const units::Velocity& end_speed)
            {
                const units::Distance length = Translation2d(m_end, m_start).norm();
                if(length > k_epsilon)
                {
                    if(!m_b.marker().empty())
                    {
                        p.addSegment(PathSegment(m_start.x(), m_start.y(), m_end.x(), m_end.y(), m_b.speed(), p.lastMotionState(), end_speed, m_b.marker()));
                    }
                    else
                    {
                        p.addSegment(PathSegment(m_start.x(), m_start.y(), m_end.x(), m_end.y(), m_b.speed(), p.lastMotionState(), end_speed));
                    }
                }
            }
        }
    }
}