#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <memory>
#include <type_traits>

#include <units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct SplineCoord;

            class Spline
            {
                virtual std::shared_ptr<SplineCoord> calculate(const Time& time) = 0;

                virtual double deriv(const Time& time) = 0;

                virtual Distance arcLength(uint samples) = 0;
            };

            template<typename T>
            typename std::enable_if< std::is_base_of<Spline, T>::value, Distance>::type distance(std::vector < std::shared_ptr<T> > splines, uint samples)
            {
                Distance d = 0;
                for (int i = 0, spline_count = splines.size(); i < spline_count; i++)
                {
                    d += splines[i].arcLength(samples);
                }
                return d;
            }
        }
    }
}

#endif // SPLINE_HPP
