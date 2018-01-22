#include <teb_planner/obstacle.hpp>

namespace rip
{
    namespace navigation
    {

        Obstacle::Obstacle()
            : m_dynamic(false)
        {}

        bool Obstacle::isDynamic() const
        {
            return m_dynamic;
        }

        void Obstacle::setCentoidVelocity(const VelocityPose& velocity)
        {
            m_velocity = velocity;
            m_dynamic = true;
        }

        VelocityPose Obstacle::getCentroidVelocity() const
        {
            return m_velocity;
        }
    }
}
