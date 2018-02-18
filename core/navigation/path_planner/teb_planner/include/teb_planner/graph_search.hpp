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

#ifndef GRAPH_SEARCH_INTERFACE_HPP
#define GRAPH_SEARCH_INTERFACE_HPP

#include <random>

#include <eigen3/Eigen/Core>

#include <teb_planner/equivalence_relations.hpp>
#include <teb_planner/pose_se2.hpp>
#include <teb_planner/teb_config.hpp>
#include <teb_planner/graph.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            class HomotopyClassPlanner; // Forward declaration


            //!< Inline function used for calculateHSignature() in combination with HCP graph vertex descriptors
            inline std::complex<long double> getCplxFromVertex(std::shared_ptr<Vertex> vertex)
            {
                return std::complex<long double>(vertex->pos().x(), vertex->pos().y());
            }

            //!< Inline function used for initializing the TEB in combination with HCP graph vertex descriptors
            inline const Eigen::Vector2d getVector2dFromVertex(std::shared_ptr<Vertex> vertex)
            {
                return vertex->pos();
            }

            /**
            * @brief Base class for graph based path planning / homotopy class sampling
            */
            class GraphSearchInterface
            {
            public:

                virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const fakeros::Twist* start_velocity) = 0;

                /**
                * @brief Clear any existing graph of the homotopy class search
                */
                void clearGraph()
                {
                    m_graph.clear();
                }

            protected:
                /**
                * @brief Protected constructor that should be called by subclasses
                */
                GraphSearchInterface(std::shared_ptr<TebConfig> cfg, HomotopyClassPlanner* hcp)
                    : m_config(cfg)
                    , m_hcp(hcp)
                {}

                /**
                * @brief Depth First Search implementation to find all paths between the start and the specified goal vertex.
                *
                * Complete paths are stored to the internal path container.
                * @sa http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/
                * @param g Graph on which the depth first should be performed
                * @param visited A container that stores visited vertices (pass an empty container, it will be filled inside during recursion).
                * @param goal Desired goal vertex
                * @param start_orientation Orientation of the first trajectory pose, required to initialize the trajectory/TEB
                * @param goal_orientation Orientation of the goal trajectory pose, required to initialize the trajectory/TEB
                * @param start_velocity start velocity (optional)
                */
                void depthFirst(Graph& g, std::vector< std::shared_ptr<Vertex> >& visited, std::shared_ptr<Vertex> goal, double start_orientation, double goal_orientation, const fakeros::Twist* start_velocity);


            protected:
                Graph m_graph; //!< Store the graph that is utilized to find alternative homotopy classes.
                std::shared_ptr<TebConfig> m_config; //!< Config class that stores and manages all related parameters
                HomotopyClassPlanner* m_hcp; //!< Raw pointer to the HomotopyClassPlanner. The HomotopyClassPlanner itself is guaranteed to outlive the graph search class it is holding.

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };



            class lrKeyPointGraph : public GraphSearchInterface
            {
            public:
                lrKeyPointGraph(std::shared_ptr<TebConfig> cfg, HomotopyClassPlanner* hcp)
                    : GraphSearchInterface(cfg, hcp)
                {}

                virtual ~lrKeyPointGraph() {}

                /**
                * @brief Create a graph containing points in the global frame that can be used to explore new possible paths between start and goal.
                *
                * This version of the graph creation places a keypoint on the left and right side of each obstacle w.r.t to the goal heading. \n
                * All feasible paths between start and goal point are extracted using a Depth First Search afterwards. \n
                * This version works very well for small point obstacles. For more complex obstacles call the createProbRoadmapGraph()
                * method that samples keypoints in a predefined area and hopefully finds all relevant alternative paths.
                *
                * @see createProbRoadmapGraph
                * @param start Start pose from wich to start on (e.g. the current robot pose).
                * @param goal Goal pose to find paths to (e.g. the robot's goal).
                * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
                * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
                * @param start_velocity start velocity (optional)
                */
                virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const fakeros::Twist* start_velocity);
            };




            class ProbRoadmapGraph : public GraphSearchInterface
            {
            public:
                ProbRoadmapGraph(std::shared_ptr<TebConfig> cfg, HomotopyClassPlanner* hcp)
                    : GraphSearchInterface(cfg, hcp)
                {}

                virtual ~ProbRoadmapGraph() {}


                /**
                * @brief Create a graph and sample points in the global frame that can be used to explore new possible paths between start and goal.
                *
                * This version of the graph samples keypoints in a predefined area (config) in the current frame between start and goal. \n
                * Afterwards all feasible paths between start and goal point are extracted using a Depth First Search. \n
                * Use the sampling method for complex, non-point or huge obstacles. \n
                * You may call createGraph() instead.
                *
                * @see createGraph
                * @param start Start pose from wich to start on (e.g. the current robot pose).
                * @param goal Goal pose to find paths to (e.g. the robot's goal).
                * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
                * @param no_samples number of random samples
                * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
                * @param start_velocity start velocity (optional)
                */
                virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const fakeros::Twist* start_velocity);

            private:
                std::mt19937 rnd_generator_; //!< Random number generator used by createProbRoadmapGraph to sample graph keypoints.
            };
        }
    }
}

#endif // GRAPH_SEARCH_INTERFACE_H
