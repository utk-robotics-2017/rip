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

#include <teb_planner/timed_elastic_band.hpp>

#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            TimedElasticBand::TimedElasticBand()
            {
            }

            TimedElasticBand::~TimedElasticBand()
            {
                // misc::Logger::getInstance()->debug("Destructor Timed_Elastic_Band...");
                clearTimedElasticBand();
            }


            void TimedElasticBand::addPose(const PoseSE2& pose, bool fixed)
            {
                VertexPose* pose_vertex = new VertexPose(pose, fixed);
                m_pose_sequence.push_back( pose_vertex );
                return;
            }

            void TimedElasticBand::addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed)
            {
                VertexPose* pose_vertex = new VertexPose(position, theta, fixed);
                m_pose_sequence.push_back( pose_vertex );
                return;
            }

            void TimedElasticBand::addPose(double x, double y, double theta, bool fixed)
            {
                VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
                m_pose_sequence.push_back( pose_vertex );
                return;
            }

            void TimedElasticBand::addTimeDiff(double dt, bool fixed)
            {
                VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
                m_time_diff_sequence.push_back( timediff_vertex );
                return;
            }


            void TimedElasticBand::addPoseAndTimeDiff(double x, double y, double angle, double dt)
            {
                if (sizePoses() != sizeTimeDiffs())
                {
                    addPose(x, y, angle, false);
                    addTimeDiff(dt, false);
                }
                else
                {
                    misc::Logger::getInstance()->error("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
                }
                return;
            }



            void TimedElasticBand::addPoseAndTimeDiff(const PoseSE2& pose, double dt)
            {
                if (sizePoses() != sizeTimeDiffs())
                {
                    addPose(pose, false);
                    addTimeDiff(dt, false);
                }
                else
                {
                    misc::Logger::getInstance()->error("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
                }
                return;
            }

            void TimedElasticBand::addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt)
            {
                if (sizePoses() != sizeTimeDiffs())
                {
                    addPose(position, theta, false);
                    addTimeDiff(dt, false);
                }
                else
                {
                    misc::Logger::getInstance()->error("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
                }
                return;
            }


            void TimedElasticBand::deletePose(int index)
            {
                assert(index < m_pose_sequence.size());
                delete m_pose_sequence.at(index);
                m_pose_sequence.erase(m_pose_sequence.begin() + index);
            }

            void TimedElasticBand::deletePoses(int index, int number)
            {
                assert(index + number <= (int)m_pose_sequence.size());
                for (int i = index; i < index + number; ++i)
                {
                    delete m_pose_sequence.at(i);
                }
                m_pose_sequence.erase(m_pose_sequence.begin() + index, m_pose_sequence.begin() + index + number);
            }

            void TimedElasticBand::deleteTimeDiff(int index)
            {
                assert(index < (int)m_time_diff_sequence.size());
                delete m_time_diff_sequence.at(index);
                m_time_diff_sequence.erase(m_time_diff_sequence.begin() + index);
            }

            void TimedElasticBand::deleteTimeDiffs(int index, int number)
            {
                assert(index + number <= m_time_diff_sequence.size());
                for (int i = index; i < index + number; ++i)
                {
                    delete m_time_diff_sequence.at(i);
                }
                m_time_diff_sequence.erase(m_time_diff_sequence.begin() + index, m_time_diff_sequence.begin() + index + number);
            }

            void TimedElasticBand::insertPose(int index, const PoseSE2& pose)
            {
                VertexPose* pose_vertex = new VertexPose(pose);
                m_pose_sequence.insert(m_pose_sequence.begin() + index, pose_vertex);
            }

            void TimedElasticBand::insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
            {
                VertexPose* pose_vertex = new VertexPose(position, theta);
                m_pose_sequence.insert(m_pose_sequence.begin() + index, pose_vertex);
            }

            void TimedElasticBand::insertPose(int index, double x, double y, double theta)
            {
                VertexPose* pose_vertex = new VertexPose(x, y, theta);
                m_pose_sequence.insert(m_pose_sequence.begin() + index, pose_vertex);
            }

            void TimedElasticBand::insertTimeDiff(int index, double dt)
            {
                VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
                m_time_diff_sequence.insert(m_time_diff_sequence.begin() + index, timediff_vertex);
            }


            void TimedElasticBand::clearTimedElasticBand()
            {
                for (PoseSequence::iterator pose_it = m_pose_sequence.begin(); pose_it != m_pose_sequence.end(); ++pose_it)
                {
                    delete *pose_it;
                }
                m_pose_sequence.clear();

                for (TimeDiffSequence::iterator dt_it = m_time_diff_sequence.begin(); dt_it != m_time_diff_sequence.end(); ++dt_it)
                {
                    delete *dt_it;
                }
                m_time_diff_sequence.clear();
            }


            void TimedElasticBand::setPoseVertexFixed(int index, bool status)
            {
                assert(index < sizePoses());
                m_pose_sequence.at(index)->setFixed(status);
            }

            void TimedElasticBand::setTimeDiffVertexFixed(int index, bool status)
            {
                assert(index < sizeTimeDiffs());
                m_time_diff_sequence.at(index)->setFixed(status);
            }


            void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
            {
                /// iterate through all TEB states and add/remove states!

                bool modified = true;

                for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
                {
                    modified = false;

                    for (int i = 0; i < sizeTimeDiffs(); ++i) // TimeDiff connects Point(i) with Point(i+1)
                    {
                        if (timeDiff(i) > dt_ref + dt_hysteresis && sizeTimeDiffs() < max_samples)
                        {
                            double newtime = 0.5 * timeDiff(i);

                            timeDiff(i) = newtime;
                            insertPose(i + 1, PoseSE2::average(pose(i), pose(i + 1)) );
                            insertTimeDiff(i + 1, newtime);

                            modified = true;
                        }
                        else if (timeDiff(i) < dt_ref - dt_hysteresis && sizeTimeDiffs() > min_samples) // only remove samples if size is larger than min_samples.
                        {
                            if (i < ((int)sizeTimeDiffs() - 1))
                            {
                                timeDiff(i + 1) = timeDiff(i + 1) + timeDiff(i);
                                deleteTimeDiff(i);
                                deletePose(i + 1);
                            }

                            modified = true;
                        }
                    }
                    if (fast_mode) { break; }
                }
            }


            double TimedElasticBand::getSumOfAllTimeDiffs() const
            {
                double time = 0;

                for (TimeDiffSequence::const_iterator dt_it = m_time_diff_sequence.begin(); dt_it != m_time_diff_sequence.end(); ++dt_it)
                {
                    time += (*dt_it)->dt();
                }
                return time;
            }

            double TimedElasticBand::getSumOfTimeDiffsUpToIdx(int index) const
            {
                assert(index <= m_time_diff_sequence.size());

                double time = 0;

                for (int i = 0; i < index; ++i)
                {
                    time += m_time_diff_sequence.at(i)->dt();
                }

                return time;
            }

            double TimedElasticBand::getAccumulatedDistance() const
            {
                double dist = 0;

                for (int i = 1; i < sizePoses(); ++i)
                {
                    dist += (pose(i).position() - pose(i - 1).position()).norm();
                }
                return dist;
            }

            bool TimedElasticBand::initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double max_vel_x, int min_samples, bool guess_backwards_motion)
            {
                if (!isInit())
                {
                    addPose(start); // add starting point
                    setPoseVertexFixed(0, true); // StartConf is a fixed constraint during optimization

                    double timestep = 0.1;

                    if (diststep != 0)
                    {
                        Eigen::Vector2d point_to_goal = goal.position() - start.position();
                        double dir_to_goal = std::atan2(point_to_goal[1], point_to_goal[0]); // direction to goal
                        double dx = diststep * std::cos(dir_to_goal);
                        double dy = diststep * std::sin(dir_to_goal);
                        double orient_init = dir_to_goal;
                        // check if the goal is behind the start pose (w.r.t. start orientation)
                        if (guess_backwards_motion && point_to_goal.dot(start.orientationUnitVec()) < 0)
                        {
                            orient_init = g2o::normalize_theta(orient_init + M_PI);
                        }
                        // TODO: timestep ~ max_vel_x_backwards for backwards motions

                        double dist_to_goal = point_to_goal.norm();
                        double no_steps_d = dist_to_goal / std::abs(diststep); // ignore negative values
                        unsigned int no_steps = (unsigned int) std::floor(no_steps_d);

                        if (max_vel_x > 0) { timestep = diststep / max_vel_x; }

                        for (unsigned int i = 1; i <= no_steps; i++) // start with 1! starting point had index 0
                        {
                            if (i == no_steps && no_steps_d == (float) no_steps)
                            {
                                break;    // if last conf (depending on stepsize) is equal to goal conf -> leave loop
                            }
                            addPoseAndTimeDiff(start.x() + i * dx, start.y() + i * dy, orient_init, timestep);
                        }

                    }

                    // if number of samples is not larger than min_samples, insert manually
                    if ( sizePoses() < min_samples - 1 )
                    {
                        misc::Logger::getInstance()->debug("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
                        while (sizePoses() < min_samples - 1) // subtract goal point that will be added later
                        {
                            // simple strategy: interpolate between the current pose and the goal
                            PoseSE2 intermediate_pose = PoseSE2::average(backPose(), goal);
                            if (max_vel_x > 0)
                            {
                                timestep = (intermediate_pose.position() - backPose().position()).norm() / max_vel_x;
                            }
                            addPoseAndTimeDiff( intermediate_pose, timestep ); // let the optimier correct the timestep (TODO: better initialization
                        }
                    }

                    // add goal
                    if (max_vel_x > 0)
                    {
                        timestep = (goal.position() - backPose().position()).norm() / max_vel_x;
                    }
                    addPoseAndTimeDiff(goal, timestep); // add goal point
                    setPoseVertexFixed(sizePoses() - 1, true); // GoalConf is a fixed constraint during optimization
                }
                else // size!=0
                {
                    misc::Logger::getInstance()->warn("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
                    misc::Logger::getInstance()->warn("Number of TEB configurations: %d, Number of TEB timediffs: %d", (unsigned int) sizePoses(), (unsigned int) sizeTimeDiffs());
                    return false;
                }
                return true;
            }


            bool TimedElasticBand::initTrajectoryToGoal(const std::vector<fakeros::PoseStamped>& plan, double max_vel_x, bool estimate_orient, int min_samples, bool guess_backwards_motion)
            {

                if (!isInit())
                {
                    PoseSE2 start(plan.front().pose);
                    PoseSE2 goal(plan.back().pose);

                    double dt = 0.1;

                    addPose(start); // add starting point with given orientation
                    setPoseVertexFixed(0, true); // StartConf is a fixed constraint during optimization

                    bool backwards = false;
                    if (guess_backwards_motion && (goal.position() - start.position()).dot(start.orientationUnitVec()) < 0) // check if the goal is behind the start pose (w.r.t. start orientation)
                    {
                        backwards = true;
                    }
                    // TODO: dt ~ max_vel_x_backwards for backwards motions

                    for (int i = 1; i < (int)plan.size() - 1; ++i)
                    {
                        double yaw;
                        if (estimate_orient)
                        {
                            // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
                            double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
                            double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
                            yaw = std::atan2(dy, dx);
                            if (backwards)
                            {
                                yaw = g2o::normalize_theta(yaw + M_PI);
                            }
                        }
                        else
                        {
                            yaw = asin(2 * plan[i].pose.orientation.x * plan[i].pose.orientation.y + 2 * plan[i].pose.orientation.z * plan[i].pose.orientation.w);
                        }
                        PoseSE2 intermediate_pose(plan[i].pose.position.x, plan[i].pose.position.y, yaw);
                        if (max_vel_x > 0)
                        {
                            dt = (intermediate_pose.position() - backPose().position()).norm() / max_vel_x;
                        }
                        addPoseAndTimeDiff(intermediate_pose, dt);
                    }

                    // if number of samples is not larger than min_samples, insert manually
                    if ( sizePoses() < min_samples - 1 )
                    {
                        misc::Logger::getInstance()->debug("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
                        while (sizePoses() < min_samples - 1) // subtract goal point that will be added later
                        {
                            // simple strategy: interpolate between the current pose and the goal
                            PoseSE2 intermediate_pose = PoseSE2::average(backPose(), goal);
                            if (max_vel_x > 0)
                            {
                                dt = (intermediate_pose.position() - backPose().position()).norm() / max_vel_x;
                            }
                            addPoseAndTimeDiff( intermediate_pose, dt ); // let the optimier correct the timestep (TODO: better initialization
                        }
                    }

                    // Now add final state with given orientation
                    if (max_vel_x > 0)
                    {
                        dt = (goal.position() - backPose().position()).norm() / max_vel_x;
                    }
                    addPoseAndTimeDiff(goal, dt);
                    setPoseVertexFixed(sizePoses() - 1, true); // GoalConf is a fixed constraint during optimization
                }
                else // size!=0
                {
                    misc::Logger::getInstance()->warn("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
                    misc::Logger::getInstance()->warn("Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
                    return false;
                }

                return true;
            }


            int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance, int begin_idx) const
            {
                std::vector<double> dist_vec; // TODO: improve! efficiency
                dist_vec.reserve(sizePoses());

                int n = sizePoses();

                // calc distances
                for (int i = begin_idx; i < n; i++)
                {
                    Eigen::Vector2d diff = ref_point - pose(i).position();
                    dist_vec.push_back(diff.norm());
                }

                if (dist_vec.empty())
                {
                    return -1;
                }

                // find minimum
                int index_min = 0;

                double last_value = dist_vec.at(0);
                for (int i = 1; i < (int)dist_vec.size(); i++)
                {
                    if (dist_vec.at(i) < last_value)
                    {
                        last_value = dist_vec.at(i);
                        index_min = i;
                    }
                }
                if (distance)
                {
                    *distance = last_value;
                }
                return begin_idx + index_min;
            }


            int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance) const
            {
                std::vector<double> dist_vec; // TODO: improve! efficiency
                dist_vec.reserve(sizePoses());

                int n = sizePoses();

                // calc distances
                for (int i = 0; i < n; i++)
                {
                    Eigen::Vector2d point = pose(i).position();
                    double diff = distance_point_to_segment_2d(point, ref_line_start, ref_line_end);
                    dist_vec.push_back(diff);
                }

                if (dist_vec.empty())
                {
                    return -1;
                }

                // find minimum
                int index_min = 0;

                double last_value = dist_vec.at(0);
                for (int i = 1; i < (int)dist_vec.size(); i++)
                {
                    if (dist_vec.at(i) < last_value)
                    {
                        last_value = dist_vec.at(i);
                        index_min = i;
                    }
                }
                if (distance)
                {
                    *distance = last_value;
                }
                return index_min; // return index, because it's equal to the vertex, which represents this bandpoint
            }

            int TimedElasticBand::findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance) const
            {
                if (vertices.empty())
                {
                    return 0;
                }
                else if (vertices.size() == 1)
                {
                    return findClosestTrajectoryPose(vertices.front());
                }
                else if (vertices.size() == 2)
                {
                    return findClosestTrajectoryPose(vertices.front(), vertices.back());
                }

                std::vector<double> dist_vec; // TODO: improve! efficiency
                dist_vec.reserve(sizePoses());

                int n = sizePoses();

                // calc distances
                for (int i = 0; i < n; i++)
                {
                    Eigen::Vector2d point = pose(i).position();
                    double diff = HUGE_VAL;
                    for (int j = 0; j < (int) vertices.size() - 1; ++j)
                    {
                        diff = std::min(diff, distance_point_to_segment_2d(point, vertices[j], vertices[j + 1]));
                    }
                    diff = std::min(diff, distance_point_to_segment_2d(point, vertices.back(), vertices.front()));
                    dist_vec.push_back(diff);
                }

                if (dist_vec.empty())
                {
                    return -1;
                }

                // find minimum
                int index_min = 0;

                double last_value = dist_vec.at(0);
                for (int i = 1; i < (int)dist_vec.size(); i++)
                {
                    if (dist_vec.at(i) < last_value)
                    {
                        last_value = dist_vec.at(i);
                        index_min = i;
                    }
                }
                if (distance)
                {
                    *distance = last_value;
                }
                return index_min; // return index, because it's equal to the vertex, which represents this bandpoint
            }


            int TimedElasticBand::findClosestTrajectoryPose(const Obstacle& obstacle, double* distance) const
            {
                const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
                if (pobst)
                {
                    return findClosestTrajectoryPose(pobst->position(), distance);
                }

                const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
                if (lobst)
                {
                    return findClosestTrajectoryPose(lobst->start(), lobst->end(), distance);
                }

                const PolygonObstacle* polyobst = dynamic_cast<const PolygonObstacle*>(&obstacle);
                if (polyobst)
                {
                    return findClosestTrajectoryPose(polyobst->vertices(), distance);
                }

                return findClosestTrajectoryPose(obstacle.getCentroid(), distance);
            }

            bool TimedElasticBand::detectDetoursBackwards(double threshold) const
            {
                if (sizePoses() < 2) { return false; }

                Eigen::Vector2d d_start_goal = backPose().position() - pose(0).position();
                d_start_goal.normalize(); // using scalar_product without normalizing vectors first result in different threshold-effects

                /// detect based on orientation
                for (int i = 0; i < sizePoses(); ++i)
                {
                    Eigen::Vector2d orient_vector(cos( pose(i).theta() ), sin( pose(i).theta() ) );
                    if (orient_vector.dot(d_start_goal) < threshold)
                    {
                        misc::Logger::getInstance()->debug("detectDetoursBackwards() - mark TEB for deletion: start-orientation vs startgoal-vec");
                        return true; // backward direction found
                    }
                }
                return false;
            }

            void TimedElasticBand::updateAndPruneTEB(const PoseSE2* new_start, const PoseSE2* new_goal, int min_samples)
            {
                // first and simple approach: change only start confs (and virtual start conf for inital velocity)
                // TEST if optimizer can handle this "hard" placement

                if (new_start && sizePoses() > 0)
                {
                    // find nearest state (using l2-norm) in order to prune the trajectory
                    // (remove already passed states)
                    double dist_cache = (new_start->position() - pose(0).position()).norm();
                    double dist;
                    int lookahead = std::min<int>( sizePoses() - min_samples, 10); // satisfy min_samples, otherwise max 10 samples

                    int nearest_idx = 0;
                    for (int i = 1; i <= lookahead; ++i)
                    {
                        dist = (new_start->position() - pose(i).position()).norm();
                        if (dist < dist_cache)
                        {
                            dist_cache = dist;
                            nearest_idx = i;
                        }
                        else { break; }
                    }

                    // prune trajectory at the beginning (and extrapolate sequences at the end if the horizon is fixed)
                    if (nearest_idx > 0)
                    {
                        // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
                        // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since Pose(0) is fixed during optimization!
                        deletePoses(1, nearest_idx);  // delete first states such that the closest state is the new first one
                        deleteTimeDiffs(1, nearest_idx); // delete corresponding time differences
                    }

                    // update start
                    pose(0) = *new_start;
                }

                if (new_goal && sizePoses() > 0)
                {
                    backPose() = *new_goal;
                }
            }


            bool TimedElasticBand::isTrajectoryInsideRegion(double radius, double max_dist_behind_robot, int skip_poses)
            {
                if (sizePoses() <= 0)
                {
                    return true;
                }

                double radius_sq = radius * radius;
                double max_dist_behind_robot_sq = max_dist_behind_robot * max_dist_behind_robot;
                Eigen::Vector2d robot_orient = pose(0).orientationUnitVec();

                for (int i = 1; i < sizePoses(); i = i + skip_poses + 1)
                {
                    Eigen::Vector2d dist_vec = pose(i).position() - pose(0).position();
                    double dist_sq = dist_vec.squaredNorm();

                    if (dist_sq > radius_sq)
                    {
                        misc::Logger::getInstance()->info("outside robot");
                        return false;
                    }

                    // check behind the robot with a different distance, if specified (or >=0)
                    if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 && dist_sq > max_dist_behind_robot_sq)
                    {
                        misc::Logger::getInstance()->info("outside robot behind");
                        return false;
                    }

                }
                return true;
            }
        }
    }
}
