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

#include <teb_planner/teb_config.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            void TebConfig::checkParameters() const
            {
                // positive backward velocity?
                if (robot.max_vel_x_backwards <= 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
                }

                // bounds smaller than penalty epsilon
                if (robot.max_vel_x <= optim.penalty_epsilon)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.max_vel_theta <= optim.penalty_epsilon)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.acc_lim_x <= optim.penalty_epsilon)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.acc_lim_theta <= optim.penalty_epsilon)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                // dt_ref and dt_hyst
                if (trajectory.dt_ref <= trajectory.dt_hysteresis)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
                }

                // min number of samples
                if (trajectory.min_samples < 3)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
                }

                // costmap obstacle behind robot
                if (obstacles.costmap_obstacles_behind_robot_dist < 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
                }

                // hcp: obstacle heading threshold
                if (hcp.obstacle_keypoint_offset >= 1 || hcp.obstacle_keypoint_offset <= 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
                }

                // carlike
                if (robot.cmd_angle_instead_rotvel && robot.wheelbase == 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
                }

                if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius == 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
                }

                // positive weight_adapt_factor
                if (optim.weight_adapt_factor < 1.0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
                }

                if (recovery.oscillation_filter_duration < 0)
                {
                    Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
                }
            }
        }
    }
}
