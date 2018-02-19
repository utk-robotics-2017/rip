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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann, Franz Albers
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/

#ifndef EDGE_DYNAMICOBSTACLE_HPP
#define EDGE_DYNAMICOBSTACLE_HPP

#include <teb_planner/g2o_types/vertex_pose.hpp>
#include <teb_planner/g2o_types/vertex_time_diff.hpp>
#include <teb_planner/g2o_types/penalties.hpp>
#include <teb_planner/g2o_types/base_teb_edges.hpp>
#include <teb_planner/obstacles.hpp>
#include <teb_planner/teb_config.hpp>
#include <teb_planner/robot_footprint_model.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            /**
             * @class EdgeDynamicObstacle
             * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
             *
             * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
             * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
             * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
             * \e weight can be set using setInformation(). \n
             * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
             * @see TebOptimalPlanner::AddEdgesDynamicObstacles
             * @remarks Do not forget to call setTebConfig(), setVertexIdx() and
             * @warning Experimental
             */
            class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, std::shared_ptr<Obstacle>, VertexPose>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeDynamicObstacle()
                    : m_t(0)
                {
                }

                /**
                 * @brief Construct edge and specify the time for its associated pose (neccessary for computeError).
                 * @param t Estimated time until current pose is reached
                 */
                EdgeDynamicObstacle(double t)
                    : m_t(t)
                {
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {

                    if (!m_config || !_measurement || !m_robot_model)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeDynamicObstacle()");
                        assert(false);
                    }

                    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

                    double dist = m_robot_model->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, m_t);

                    _error[0] = penaltyBoundFromBelow(dist, m_config->obstacles.min_obstacle_dist, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundFromBelow(dist, m_config->obstacles.dynamic_obstacle_inflation_dist, 0.0);


                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error("EdgeDynamicObstacle::computeError() _error[0]=%f\n", _error[0]);
                        assert(false);
                    }
                }


                /**
                 * @brief Set Obstacle for the underlying cost function
                 * @param obstacle Const pointer to an Obstacle or derived Obstacle
                 */
                void setObstacle(std::shared_ptr<Obstacle> obstacle)
                {
                    _measurement = obstacle;
                }

                /**
                 * @brief Set pointer to the robot model
                 * @param robot_model Robot model required for distance calculation
                 */
                void setRobotModel(std::shared_ptr<BaseRobotFootprintModel> robot_model)
                {
                    m_robot_model = robot_model;
                }

                /**
                 * @brief Set all parameters at once
                 * @param cfg TebConfig class
                 * @param robot_model Robot model required for distance calculation
                 * @param obstacle 2D position vector containing the position of the obstacle
                 */
                void setParameters(std::shared_ptr<TebConfig> cfg, std::shared_ptr<BaseRobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle)
                {
                    m_config = cfg;
                    m_robot_model = robot_model;
                    _measurement = obstacle;
                }

            protected:
                std::shared_ptr<BaseRobotFootprintModel> m_robot_model; //!< Store pointer to robot_model
                double m_t; //!< Estimated time until current pose is reached

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };
        }
    }
}
#endif
