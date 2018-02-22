#include <spline_planner/spline_utils.hpp>

namespace rip
{
    namespace navigation
    {
        namespace splineutils
        {
            Distance computeXDiff(const Point& p1, const Point& p2, double alpha)
            {
                if (alpha == 0)
                {
                    return 1;
                }
                else
                {
                    double distanceSq = std::pow(p1.distance(p2)(), 2);

                    //if these points are right on top of each other, don't bother with the power calculation
                    if (distanceSq < .0001)
                    {
                        return 0;
                    }
                    else
                    {
                        //multiply alpha by 0.5 so that we tke the square root of distanceSq
                        //ie: result = distance ^ alpha, and distance = (distanceSq)^(0.5)
                        //so: result = (distanceSq^0.5)^(alpha) = (distanceSq)^(0.5*alpha)
                        //this way we don't have to do a pow AND a sqrt
                        return std::pow(distanceSq, alpha * 0.5);
                    }
                }
            }

            std::vector<double> computeXValuesWithInnerPadding(const std::vector<Waypoint>& points, double alpha, size_t innerPadding)
            {
                size_t size = points.size();
                size_t endPaddingIndex = size - 1 - innerPadding;
                size_t desiredMaxT = size - 2 * innerPadding - 1;

                std::vector<double> tValues(size);

                //we know points[padding] will have a t value of 0
                tValues[innerPadding] = 0;

                //loop backwards from padding to give the earlier points negative t values
                for (size_t i = innerPadding; i > 0; i--)
                {
                    //Points inside the padding will not be interpolated
                    //so give it a negative t value, so that the first actual point can have a t value of 0
                    tValues[i - 1] = tValues[i] - computeXDiff(points[i - 1].position(), points[i].position(), alpha)();
                }

                //compute the t values of the other points
                for (size_t i = 1; i < size - innerPadding; i++)
                {
                    tValues[i + innerPadding] = tValues[i - 1 + innerPadding] + computeXDiff(points[i].position(), points[i - 1].position(), alpha)();
                }

                //we want to know the t value of the last segment so that we can normalize them all
                double maxTRaw = tValues[endPaddingIndex];

                //now that we have all ouf our t values and indexes figured out, normalize the t values by dividing them by maxT
                double multiplier = desiredMaxT / maxTRaw;
                for (auto& entry : tValues)
                {
                    entry *= multiplier;
                }

                return tValues;
            }


            size_t getIndexForX(const std::vector<Distance>& knotData, const Distance& t)
            {
                //we want to find the segment whos t0 and t1 values bound x

                //if no segments bound x, return -1
                if (t <= knotData.front())
                {
                    return 0;
                }
                if (t >= knotData.back())
                {
                    return knotData.size() - 1;
                }

                //our initial guess will be to subtract the minimum t value, then take the floor
                size_t currentIndex = std::floor(t() - knotData.front()());
                size_t size = knotData.size();

                //move left or right in the array until we've found the correct index
                size_t searchSize = 1;
                while (t < knotData[currentIndex])
                {
                    while (currentIndex >= 0 && t < knotData[currentIndex])
                    {
                        searchSize++;
                        currentIndex -= searchSize;
                    }
                    if (currentIndex < 0 || t > knotData[currentIndex + 1])
                    {
                        currentIndex += searchSize;
                        searchSize /= 4;
                    }

                }
                while (t >= knotData[currentIndex + 1])
                {
                    while (currentIndex < size && t >= knotData[currentIndex])
                    {
                        searchSize++;
                        currentIndex += searchSize;
                    }
                    if (currentIndex >= size || t < knotData[currentIndex])
                    {
                        currentIndex -= searchSize;
                        searchSize /= 4;
                    }
                }
                return currentIndex;
            }
        } // namespace splineutils
    } // namespace navigation
} // namespace rip
