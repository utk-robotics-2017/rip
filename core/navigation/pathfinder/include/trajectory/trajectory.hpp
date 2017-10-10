#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <memory>
#include <vector>

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct CoupledSegment;

            template <typename SplineType, typename ProfileType>
            class Trajectory
            {
            public:
                Trajectory(Distance wheelbase);

                void configurePath(const std::vector< std::shared_ptr<SplineType> >& splines, uint spline_samples)
                {
                    m_splines = splines;
                    m_spline_samples = spline_samples;
                }

                void configureProfile(std::shared_ptr<ProfileType> profile);

                std::shared_ptr< CoupledSegment > calculate(std::shared_ptr< CoupledSegment > previous, Time time);
            private:
                std::vector< std::shared_ptr<SplineType> > m_splines;
                uint m_spline_samples;

                std::shared_ptr<ProfileType> m_profile;
            };
        }
    }
}

#endif // TRAJECTORY_HPP