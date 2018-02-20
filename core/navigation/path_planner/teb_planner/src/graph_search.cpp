/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
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
 * Authors: Christoph Rösmann, Franz Albers
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/

#include <teb_planner/graph_search.hpp>
#include <teb_planner/homotopy_class_planner.hpp>

#include <limits>
#include <functional>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            void GraphSearchInterface::depthFirst(Graph& g, std::vector< std::shared_ptr<Vertex> >& visited, std::shared_ptr<Vertex> goal, double start_orientation,
                                                  double goal_orientation, const fakeros::Twist* start_velocity)
            {
                // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths

                if (static_cast<int>(m_hcp->getTrajectoryContainer().size()) >= m_config->hcp.max_number_classes)
                {
                    return;    // We do not need to search for further possible alternative homotopy classes.
                }

                std::shared_ptr<Vertex> back = visited.back();
                std::vector< std::shared_ptr<Vertex> > adjacent_vertices = back->adjacentVertices();

                /// Examine adjacent nodes
                for (std::vector< std::shared_ptr<Vertex> >::iterator it = adjacent_vertices.begin(), end = adjacent_vertices.end(); it != end; ++it)
                {
                    if ( std::find(visited.begin(), visited.end(), *it) != visited.end() )
                    {
                        continue;    // already visited
                    }

                    if ( *it == goal ) // goal reached
                    {
                        visited.push_back(*it);

                        // Add new TEB, if this path belongs to a new homotopy class
                        m_hcp->addAndInitNewTeb(visited.begin(), visited.end(), std::bind(getVector2dFromVertex, std::placeholders::_1),
                                                start_orientation, goal_orientation, start_velocity);

                        visited.pop_back();
                        break;
                    }
                }

                /// Recursion for all adjacent vertices
                for ( std::vector< std::shared_ptr<Vertex> >::iterator it = adjacent_vertices.begin(), end = adjacent_vertices.end(); it != end; ++it)
                {
                    if ( std::find(visited.begin(), visited.end(), *it) != visited.end() || *it == goal)
                    {
                        continue;    // already visited || goal reached
                    }


                    visited.push_back(*it);

                    // recursion step
                    depthFirst(g, visited, goal, start_orientation, goal_orientation, start_velocity);

                    visited.pop_back();
                }
            }



            void lrKeyPointGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const fakeros::Twist* start_velocity)
            {
                // Clear existing graph and paths
                clearGraph();

                // Direction-vector between start and goal and normal-vector:
                Eigen::Vector2d diff = goal.position() - start.position();

                if (diff.norm() < m_config->goal_tolerance.xy_goal_tolerance)
                {
                    misc::Logger::getInstance()->debug("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
                    if (m_hcp->getTrajectoryContainer().empty())
                    {
                        misc::Logger::getInstance()->info("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
                        m_hcp->addAndInitNewTeb(start, goal, start_velocity);
                    }
                    return;
                }

                Eigen::Vector2d normal(-diff[1], diff[0]); // normal-vector
                normal.normalize();
                normal = normal * dist_to_obst; // scale with obstacle_distance;

                // Insert Vertices
                std::shared_ptr<Vertex> start_vtx = m_graph.addVertex(start.position()); // start vertex
                diff.normalize();

                // store nearest obstacle keypoints -> only used if limit_obstacle_heading is enabled
                std::pair< std::shared_ptr<Vertex>, std::shared_ptr<Vertex> > nearest_obstacle; // both vertices are stored
                double min_dist = std::numeric_limits<double>::max();

                if (m_hcp->obstacles() != nullptr)
                {
                    for (ObstacleContainer::const_iterator it_obst = m_hcp->obstacles()->begin(); it_obst != m_hcp->obstacles()->end(); ++it_obst)
                    {
                        // check if obstacle is placed in front of start point
                        Eigen::Vector2d start2obst = (*it_obst)->getCentroid() - start.position();
                        double dist = start2obst.norm();
                        if (start2obst.dot(diff) / dist < 0.1)
                        {
                            continue;
                        }

                        // Add Keypoints
                        std::shared_ptr<Vertex> u = m_graph.addVertex((*it_obst)->getCentroid() + normal);
                        std::shared_ptr<Vertex> v = m_graph.addVertex((*it_obst)->getCentroid() - normal);

                        // store nearest obstacle
                        if (obstacle_heading_threshold && dist < min_dist)
                        {
                            min_dist = dist;
                            nearest_obstacle.first = u;
                            nearest_obstacle.second = v;
                        }
                    }
                }

                std::shared_ptr<Vertex> goal_vtx = m_graph.addVertex(goal.position());

                // Insert Edges
                std::vector< std::shared_ptr<Vertex> > vertices = m_graph.vertices();
                for (std::vector< std::shared_ptr< Vertex > >::iterator it_i = vertices.begin(), end_i = vertices.end(); it_i != end_i - 1; ++it_i) // ignore goal in this loop
                {
                    for (std::vector< std::shared_ptr< Vertex > >::iterator it_j = vertices.begin(), end_j = vertices.end(); it_j != end_j; ++it_j) // check all forward connections
                    {
                        if (it_i == it_j)
                        {
                            continue;
                        }
                        // TODO: make use of knowing in which order obstacles are inserted and that for each obstacle 2 vertices are added,
                        // therefore we must only check one of them.
                        Eigen::Vector2d distij = (*it_j)->pos() - (*it_i)->pos();
                        distij.normalize();
                        // Check if the direction is backwards:
                        if (distij.dot(diff) <= obstacle_heading_threshold)
                        {
                            continue;
                        }


                        // Check start angle to nearest obstacle
                        if (obstacle_heading_threshold && *it_i == start_vtx && min_dist != std::numeric_limits<double>::max())
                        {
                            if (*it_j == nearest_obstacle.first || *it_j == nearest_obstacle.second)
                            {
                                Eigen::Vector2d keypoint_dist = (*it_j)->pos() - start.position();
                                keypoint_dist.normalize();
                                Eigen::Vector2d start_orient_vec( cos(start.theta()), sin(start.theta()) ); // already normalized
                                // check angle
                                if (start_orient_vec.dot(keypoint_dist) <= obstacle_heading_threshold)
                                {
                                    misc::Logger::getInstance()->debug("createGraph() - deleted edge: limit_obstacle_heading");
                                    continue;
                                }
                            }
                        }

                        // Collision Check

                        if (m_hcp->obstacles() != nullptr)
                        {
                            bool collision = false;
                            for (ObstacleContainer::const_iterator it_obst = m_hcp->obstacles()->begin(); it_obst != m_hcp->obstacles()->end(); ++it_obst)
                            {
                                if ( (*it_obst)->checkLineIntersection((*it_i)->pos(), (*it_j)->pos(), 0.5 * dist_to_obst) )
                                {
                                    collision = true;
                                    break;
                                }
                            }
                            if (collision)
                            {
                                continue;
                            }
                        }

                        // Create Edge
                        m_graph.addEdge(*it_i, *it_j);
                    }
                }


                // Find all paths between start and goal!
                std::vector< std::shared_ptr<Vertex> > visited;
                visited.push_back(start_vtx);
                depthFirst(m_graph, visited, goal_vtx, start.theta(), goal.theta(), start_velocity);
            }



            void ProbRoadmapGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const fakeros::Twist* start_velocity)
            {
                // Clear existing graph and paths
                clearGraph();

                // Direction-vector between start and goal and normal-vector:
                Eigen::Vector2d diff = goal.position() - start.position();
                double start_goal_dist = diff.norm();

                if (start_goal_dist < m_config->goal_tolerance.xy_goal_tolerance)
                {
                    misc::Logger::getInstance()->debug("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
                    if (m_hcp->getTrajectoryContainer().empty())
                    {
                        misc::Logger::getInstance()->info("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
                        m_hcp->addAndInitNewTeb(start, goal, start_velocity);
                    }
                    return;
                }
                Eigen::Vector2d normal(-diff.coeffRef(1), diff.coeffRef(0)); // normal-vector
                normal.normalize();

                // Now sample vertices between start, goal and a specified width between both sides
                // Let's start with a square area between start and goal (maybe change it later to something like a circle or whatever)

                double area_width = m_config->hcp.roadmap_graph_area_width;

                std::uniform_real_distribution<double> distribution_x(0, start_goal_dist * m_config->hcp.roadmap_graph_area_length_scale);
                std::uniform_real_distribution<double> distribution_y(0, area_width);

                double phi = atan2(diff.coeffRef(1), diff.coeffRef(0)); // rotate area by this angle
                Eigen::Rotation2D<double> rot_phi(phi);

                Eigen::Vector2d area_origin;
                if (m_config->hcp.roadmap_graph_area_length_scale != 1.0)
                {
                    area_origin = start.position() + 0.5 * (1.0 - m_config->hcp.roadmap_graph_area_length_scale) * start_goal_dist * diff.normalized() - 0.5 * area_width * normal;    // bottom left corner of the origin
                }
                else
                {
                    area_origin = start.position() - 0.5 * area_width * normal;    // bottom left corner of the origin
                }

                // Insert Vertices
                std::shared_ptr<Vertex> start_vtx = m_graph.addVertex(start.position());
                diff.normalize(); // normalize in place


                // Start sampling
                for (int i = 0; i < m_config->hcp.roadmap_graph_no_samples; ++i)
                {
                    Eigen::Vector2d sample;
                    // Sample coordinates
                    sample = area_origin + rot_phi * Eigen::Vector2d(distribution_x(rnd_generator_), distribution_y(rnd_generator_));



                    // Add new vertex
                    std::shared_ptr<Vertex> v = m_graph.addVertex(sample);
                }

                // Now add goal vertex
                std::shared_ptr<Vertex> goal_vtx = m_graph.addVertex(goal.position());


                // Insert Edges
                std::vector< std::shared_ptr<Vertex> > vertices = m_graph.vertices();
                for (std::vector< std::shared_ptr< Vertex > >::iterator it_i = vertices.begin(), end_i = vertices.end(); it_i != end_i - 1; ++it_i) // ignore goal in this loop
                {
                    for (std::vector< std::shared_ptr< Vertex > >::iterator it_j = vertices.begin(), end_j = vertices.end(); it_j != end_j; ++it_j) // check all forward connections
                    {
                        if (it_i == it_j) // same vertex found
                        {
                            continue;
                        }

                        Eigen::Vector2d distij = (*it_j)->pos() - (*it_i)->pos();
                        distij.normalize(); // normalize in place

                        // Check if the direction is backwards:
                        if (distij.dot(diff) <= obstacle_heading_threshold)
                        {
                            continue;    // diff is already normalized
                        }


                        // Collision Check
                        bool collision = false;
                        for (ObstacleContainer::const_iterator it_obst = m_hcp->obstacles()->begin(); it_obst != m_hcp->obstacles()->end(); ++it_obst)
                        {
                            if ( (*it_obst)->checkLineIntersection((*it_i)->pos(), (*it_j)->pos(), dist_to_obst) )
                            {
                                collision = true;
                                break;
                            }
                        }
                        if (collision)
                        {
                            continue;
                        }

                        // Create Edge
                        m_graph.addEdge(*it_i, *it_j);
                    }
                }

                /// Find all paths between start and goal!
                std::vector< std::shared_ptr<Vertex> > visited;
                visited.push_back(start_vtx);
                depthFirst(m_graph, visited, goal_vtx, start.theta(), goal.theta(), start_velocity);
            }
        }
    }
}
