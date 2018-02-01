/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/

#include <teb_planner/obstacles.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            void PolygonObstacle::fixPolygonClosure()
            {
                if (m_vertices.size() < 2)
                {
                    return;
                }

                if (m_vertices.front().isApprox(m_vertices.back()))
                {
                    m_vertices.pop_back();
                }
            }

            void PolygonObstacle::calcCentroid()
            {
                if (m_vertices.empty())
                {
                    m_centroid.setConstant(NAN);
                    Logger::getInstance()->warn("PolygonObstacle::calcCentroid(): number of vertices is empty. the resulting centroid is a vector of NANs.");
                    return;
                }

                // if polygon is a point
                if (noVertices() == 1)
                {
                    m_centroid = m_vertices.front();
                    return;
                }

                // if polygon is a line:
                if (noVertices() == 2)
                {
                    m_centroid = 0.5 * (m_vertices.front() + m_vertices.back());
                    return;
                }

                // otherwise:

                m_centroid.setZero();

                // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
                double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
                for (int i = 0; i < noVertices() - 1; ++i)
                {
                    A += m_vertices.at(i).coeffRef(0) * m_vertices.at(i + 1).coeffRef(1) - m_vertices.at(i + 1).coeffRef(0) * m_vertices.at(i).coeffRef(1);
                }
                A += m_vertices.at(noVertices() - 1).coeffRef(0) * m_vertices.at(0).coeffRef(1) - m_vertices.at(0).coeffRef(0) * m_vertices.at(noVertices() - 1).coeffRef(1);
                A *= 0.5;

                if (A != 0)
                {
                    for (int i = 0; i < noVertices() - 1; ++i)
                    {
                        double aux = (m_vertices.at(i).coeffRef(0) * m_vertices.at(i + 1).coeffRef(1) - m_vertices.at(i + 1).coeffRef(0) * m_vertices.at(i).coeffRef(1));
                        m_centroid +=  ( m_vertices.at(i) + m_vertices.at(i + 1) ) * aux;
                    }
                    double aux = (m_vertices.at(noVertices() - 1).coeffRef(0) * m_vertices.at(0).coeffRef(1) - m_vertices.at(0).coeffRef(0) * m_vertices.at(noVertices() - 1).coeffRef(1));
                    m_centroid +=  ( m_vertices.at(noVertices() - 1) + m_vertices.at(0) ) * aux;
                    m_centroid /= (6 * A);
                }
                else // A == 0 -> all points are placed on a 'perfect' line
                {
                    // seach for the two outer points of the line with the maximum distance inbetween
                    int i_cand = 0;
                    int j_cand = 0;
                    double min_dist = std::numeric_limits<double>::max();
                    for (int i = 0; i < noVertices(); ++i)
                    {
                        for (int j = i + 1; j < noVertices(); ++j) // start with j=i+1
                        {
                            double dist = (m_vertices[j] - m_vertices[i]).norm();
                            if (dist < min_dist)
                            {
                                min_dist = dist;
                                i_cand = i;
                                j_cand = j;
                            }
                        }
                    }
                    // calc centroid of that line
                    m_centroid = 0.5 * (m_vertices[i_cand] + m_vertices[j_cand]);
                }
            }

            Eigen::Vector2d PolygonObstacle::getClosestPoint(const Eigen::Vector2d& position) const
            {
                // the polygon is a point
                if (noVertices() == 1)
                {
                    return m_vertices.front();
                }

                if (noVertices() > 1)
                {

                    Eigen::Vector2d new_pt = closest_point_on_line_segment_2d(position, m_vertices.at(0), m_vertices.at(1));

                    if (noVertices() > 2) // real polygon and not a line
                    {
                        double dist = (new_pt - position).norm();
                        Eigen::Vector2d closest_pt = new_pt;

                        // check each polygon edge
                        for (int i = 1; i < noVertices() - 1; ++i) // skip the first one, since we already checked it (new_pt)
                        {
                            new_pt = closest_point_on_line_segment_2d(position, m_vertices.at(i), m_vertices.at(i + 1));
                            double new_dist = (new_pt - position).norm();
                            if (new_dist < dist)
                            {
                                dist = new_dist;
                                closest_pt = new_pt;
                            }
                        }
                        // afterwards we check the edge between goal and start (close polygon)
                        new_pt = closest_point_on_line_segment_2d(position, m_vertices.back(), m_vertices.front());
                        double new_dist = (new_pt - position).norm();
                        if (new_dist < dist)
                        {
                            return new_pt;
                        }
                        else
                        {
                            return closest_pt;
                        }
                    }
                    else
                    {
                        return new_pt; // closest point on line segment
                    }
                }

                Logger::getInstance()->error("PolygonObstacle::getClosestPoint() cannot find any closest point. Polygon ill-defined?");
                return Eigen::Vector2d::Zero(); // todo: maybe boost::optional?
            }

            bool PolygonObstacle::checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist) const
            {
                // Simple strategy, check all edge-line intersections until an intersection is found...
                // check each polygon edge
                for (int i = 0; i < noVertices() - 1; ++i)
                {
                    if ( check_line_segments_intersection_2d(line_start, line_end, m_vertices.at(i), m_vertices.at(i + 1)) )
                    {
                        return true;
                    }
                }
                if (noVertices() == 2) // if polygon is a line
                {
                    return false;
                }

                return check_line_segments_intersection_2d(line_start, line_end, m_vertices.back(), m_vertices.front()); //otherwise close polygon
            }
        }
    }
}
