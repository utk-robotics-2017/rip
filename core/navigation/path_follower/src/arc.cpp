#include "path_follower/arc.hpp"
#include "path_follower/line.hpp"
#include "path_follower/epsilon.hpp"
#include "path_follower/rigid_transform_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Arc::Arc(const Waypoint& a, const Waypoint& b, const Waypoint& c)
                : m_a(a, b)
                , m_b(b, c)
            {
                m_speed = (m_a.speed() + m_b.speed()) / 2.0;
                m_center = intersect(m_a, m_b);
                m_radius = Translation2d(m_center, m_a.end()).norm();
            }

            Arc::Arc(const Line& a, const Line& b)
                    : m_a(a), m_b(b), m_center(intersect(a, b)), m_speed((a.speed() + b.speed()) / 2.0)
            {
                m_radius = Translation2d(m_center, a.end()).norm();
            }

            void Arc::addToPath(Path& p)
            {
                m_a.addToPath(p, m_speed);
                if(m_radius > k_epsilon && m_radius < 1e6)
                {
                    p.addSegment(PathSegment(m_a.end().x(), m_a.end().y(), m_b.start().x(), m_b.end().y(), m_center.x(), m_center.y(), m_speed, p.lastMotionState(), m_b.speed()));
                }
            }

            Translation2d Arc::intersect(const Line& l1, const Line& l2)
            {
                const RigidTransform2d line_a(l1.end(), Rotation2d(l1.slope(), true).normal());
                const RigidTransform2d line_b(l2.start(), Rotation2d(l2.slope(), true).normal());
                return line_a.intersection(line_b);
            }
        }
    }
}