#ifndef SPLINE_UTILS_HPP
#define SPLINE_UTILS_HPP

#include <unordered_map>
#include <vector>
#include <cmath>

#include <geometry/point.hpp>

#include "waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace splineutils
        {
            /**
             * Compute the X values for the given points with the given alpha.
             * The distance between adjacent points is the magnitude of the
             * distance, raised to the power alpha
             */
            Distance computeXDiff(const Point& p1, const Point& p2, double alpha);

            /**
             * Compute the X values for the given points, based on the alpha value
             * If \p inner_padding > 0, the first 'innner_padding - 1'  values will
             * be negative, and the inner_padding'th value will be 0 so the spline
             * will effectively begin at 'inner_padding + 1'
             */
            std::vector<double> computeXValuesWithInnerPadding(const std::vector<Waypoint>& points, double alpha, size_t inner_padding);

            /**
             * Given a list of knots and an 'x' value return the index of the knot that the x value falls within
             */
            size_t getIndexForX(const std::vector<Distance>& knots, const Distance& x);
        } // namespace splineutils
    } // namespace navigation
} // namespace rip

#endif // SPLINE_UTILS_HPP
