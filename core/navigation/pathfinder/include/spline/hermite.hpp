#ifndef HERMITE_HPP
#define HERMITE_HPP

#include "spline.hpp"
#include "waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            class Hermite : public Spline
            {
            public:
                enum class Type
                {
                    kCubic,
                    kQuintic
                }; // Type

                Hermite(Type type, std::shared_ptr<Waypoint> start, std::shared_ptr<Waypoint> end);

                std::shared_ptr<SplineCoord> calculated(const Time& time);

                virtual double deriv(const Time& time) override;

                virtual Distance arcLength(uint samples) override;
            private:
                Type m_type;
                std::shared_ptr<Waypoint> m_start;
                std::shared_ptr<Waypoint> m_end;
                Distance m_x_offset;
                Distance m_y_offset;
                Angle m_angle_offset;
                Distance m_hyp_distance;
                double m_tangent0;
                double m_tangent1;
                double a;
                double b;

                double m_last_arc_calc;
                int m_last_arc_calc_samples;
            }; // class Hermite

            std::vector< std::shared_ptr<Hermite> > hermite(Hermite::Type tpye, const std::vector< std::shared_ptr<Waypoint> >& waypooints);
        }
    }
} // namespace rip

#endif // HERMITE_HPP
