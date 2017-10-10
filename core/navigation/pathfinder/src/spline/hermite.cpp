#include "pathfinder/spline/hermite.hpp"
#include "pathfinder/spline/spline_coord.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            Hermite::Hermite(Type type, std::shared_ptr<Waypoint> start, std::shared_ptr<Waypoint> end)
                : m_start(start)
                , m_end(end)
                , m_last_arc_calc_samples(0)
            {
                m_x_offset = start->x;
                m_y_offset = start->y;

                Distance dy = end->y - start->y;
                Distance dx = end->x - start->x;
                m_angle_offset = atan2(dy(), dx());
                m_hyp_distance = sqrt((dx * dx)() + (dy * dy)());

                m_tangent0 = tan((start->theta - m_angle_offset)());
                m_tangent1 = tan((end->theta - m_angle_offset)());
            }

            std::shared_ptr<SplineCoord> Hermite::calculated(const Time& time)
            {
                std::shared_ptr<SplineCoord> coord = std::make_shared<SplineCoord>();
                coord->time = time;

                Distance x = m_hyp_distance * time();
                Distance y = 0;

                if (m_type == Type::kCubic)
                {
                    y = ((m_tangent0 + m_tangent1) * m_hyp_distance * pow(time(), 3))
                        + (-(2 * m_tangent0 + m_tangent1) * m_hyp_distance * pow(time(), 2))
                        + m_tangent0 * m_hyp_distance * time();
                }
                else if (m_type == Type::kQuintic)
                {
                    y = (-(3 * (m_tangent0 + m_tangent1))) * m_hyp_distance * pow(time(), 5)
                        + (8 * m_tangent0 + 7 * m_tangent1) * m_hyp_distance * pow(time(), 4)
                        + (-(6 * m_tangent0 + 4 * m_tangent1)) * m_hyp_distance * pow(time(), 3)
                        + (m_tangent0) * m_hyp_distance * time();
                }

                coord->x = x;
                coord->y = y;
                coord->angle = atan(deriv(time())) * rad + m_angle_offset;

            }

            double Hermite::deriv(const Time& time)
            {
                double x = m_hyp_distance() * time();
                if (m_type == Type::kCubic)
                {
                    return (3 * (m_tangent0 + m_tangent1) * pow(time(), 2))
                           + (2 * (-(2 * m_tangent0 + m_tangent1) * time()))
                           + m_tangent0;
                }
                else if (m_type == Type::kQuintic)
                {
                    return 5 * (-(3 * (m_tangent0 + m_tangent1))) * pow(time(), 4)
                           + 4 * (8 * m_tangent0 + 7 * m_tangent1) * pow(time(), 3)
                           + 3 * (-(6 * m_tangent0 + 4 * m_tangent1)) * pow(time(), 2)
                           + m_tangent0;
                }
            }

            Distance Hermite::arcLength(uint samples)
            {
                if (m_last_arc_calc_samples != samples)
                {
                    double t = 0, dt = (1.0f / samples);

                    double dydt = deriv(t);
                    double integrand = 0;
                    double arc_length = 0;
                    double last_integrand = sqrt(1 + dydt * dydt) * dt;

                    for (t = 0; t <= 1; t += dt)
                    {
                        dydt = deriv(t);
                        integrand = sqrt(1 + dydt * dydt) * dt;
                        arc_length += (integrand + last_integrand) / 2;
                        last_integrand = integrand;
                    }
                    double al = m_hyp_distance() * arc_length;
                    m_last_arc_calc_samples = samples;
                    m_last_arc_calc = al;
                }
                return m_last_arc_calc;
            }
        }
    }
}