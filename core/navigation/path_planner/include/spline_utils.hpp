#ifndef SPLINE_UTILS_HPP
#define SPLINE_UTILS_HPP

#include <unordered_map>
#include <vector>
#include <cmath>

#include "point.hpp"
#include "waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
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

                template <typename Function>
                Distance gaussLegendreQuadratureIntegral(Function f)
                {
                    const size_t NUM_POINTS = 13;

                    //these are precomputed :( It would be cool to compute these at compile time, but apparently
                    //it's not easy to compute the points/weights just given the number of points.
                    //it involves computing every root of a polynomial. which can obviously be done, but not in a reasonable amount of code
                    std::array<double, NUM_POINTS> quadraturePoints =
                    {
                        0.0000000000000000,
                        -0.2304583159551348,
                        0.2304583159551348,
                        -0.4484927510364469,
                        0.4484927510364469,
                        -0.6423493394403402,
                        0.6423493394403402,
                        -0.8015780907333099,
                        0.8015780907333099,
                        -0.9175983992229779,
                        0.9175983992229779,
                        -0.9841830547185881,
                        0.9841830547185881
                    };

                    std::array<double, NUM_POINTS> quadratureWeights =
                    {
                        0.2325515532308739,
                        0.2262831802628972,
                        0.2262831802628972,
                        0.2078160475368885,
                        0.2078160475368885,
                        0.1781459807619457,
                        0.1781459807619457,
                        0.1388735102197872,
                        0.1388735102197872,
                        0.0921214998377285,
                        0.0921214998377285,
                        0.0404840047653159,
                        0.0404840047653159
                    };

                    double halfDiff = 1.0 / 2;
                    double halfSum = 1.0 / 2;

                    Distance sum;
                    for (size_t i = 0; i < NUM_POINTS; i++)
                    {
                        sum += quadratureWeights[i] * f(halfDiff * quadraturePoints[i] + halfSum);
                    }
                    return halfDiff * sum;
                }
            } // namespace splineutils
        } // namespace pathplanner
    } // namespace navigation
} // namespace rip

#endif // SPLINE_UTILS_HPP
