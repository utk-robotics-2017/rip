#include <teb_planner/timed_elastic_band.hpp>

#include <g2o/stuff/misc.h>

#include <geometry/geometry_utils.hpp>

#include <teb_planner/point_obstacle.hpp>
#include <teb_planner/line_obstacle.hpp>
#include <teb_planner/polygon_obstacle.hpp>

namespace rip
{
    namespace navigation
    {

        TimedElasticBand::~TimedElasticBand()
        {
            clear();
        }

        bool TimedElasticBand::isInit() const
        {
            return m_time_diffs.size() && m_poses.size();
        }

        std::vector<std::shared_ptr<rip::navigation::VertexPose> > rip::navigation::TimedElasticBand::poses() const
        {
            return m_poses;
        }

        rip::navigation::Pose rip::navigation::TimedElasticBand::pose(int index) const
        {
            assert(index < m_poses.size());
            return m_poses[index]->pose();
        }

        std::vector<std::shared_ptr<rip::navigation::VertexTimeDiff> > rip::navigation::TimedElasticBand::timeDiffs() const
        {
            return m_time_diffs;
        }

        units::Time TimedElasticBand::timeDiff(int index) const
        {
            assert(index < m_time_diffs.size());
            return m_time_diffs[index]->dt();
        }

        std::shared_ptr<VertexPose> TimedElasticBand::poseVertex(int index) const
        {
            assert(index < m_poses.size());
            return m_poses[index];
        }

        std::shared_ptr<VertexTimeDiff> TimedElasticBand::timeDiffVertex(int index) const
        {
            assert(index < m_time_diffs.size());
            return m_time_diffs[index];
        }

        void TimedElasticBand::addPose(const Pose& pose, bool fixed)
        {
            std::shared_ptr<VertexPose> pose_vertex = std::make_shared<VertexPose>(pose, fixed);
            m_poses.push_back(pose_vertex);
        }

        void TimedElasticBand::addTimeDiff(const units::Time& dt, bool fixed)
        {
            std::shared_ptr<VertexTimeDiff> time_diff_vertex = std::make_shared<VertexTimeDiff>(dt, fixed);
            m_time_diffs.push_back(time_diff_vertex);
        }

        void TimedElasticBand::addPoseAndTimeDiff(const Pose& pose, const units::Time& dt)
        {
            if(size() != sizeTD())
            {
                addPose(pose);
                addTimeDiff(dt);
            }
            else
            {
                // todo: throw exception
            }
        }

        void TimedElasticBand::insertPose(int index, const Pose& pose)
        {
            std::shared_ptr<VertexPose> pose_vertex = std::make_shared<VertexPose>(pose);
            m_poses.insert(m_poses.begin() + index, pose_vertex);
        }

        void TimedElasticBand::insertTimeDiff(int index, const units::Time& dt)
        {
            std::shared_ptr<VertexTimeDiff> time_diff_vertex = std::make_shared<VertexTimeDiff>(dt);
            m_time_diffs.insert(m_time_diffs.begin() + index, time_diff_vertex);
        }

        void TimedElasticBand::removePose(int index)
        {
            assert(index < m_poses.size());
            m_poses.erase(m_poses.begin() + index);
        }

        void TimedElasticBand::removePoses(int index, int number)
        {
            assert(index + number < m_poses.size());
            m_poses.erase(m_poses.begin() + index, m_poses.begin() + index + number);
        }

        void TimedElasticBand::removeTimeDiff(int index)
        {
            assert(index < m_time_diffs.size());
            m_time_diffs.erase(m_time_diffs.begin() + index);
        }

        void TimedElasticBand::removeTimeDiffs(int index, int number)
        {
            assert(index + number < m_time_diffs.size());
            m_time_diffs.erase(m_time_diffs.begin() + index, m_time_diffs.begin() + index + number);
        }

        void TimedElasticBand::initTrajectoryToGoal(const Pose& start, const Pose& goal, const units::Distance& distance_step, const units::Velocity& max_velocity, int min_samples, bool backwards_motion)
        {
            if(!isInit())
            {
                addPose(start, true);

                units::Time timestep = 0.1 * units::s;

                if(distance_step != 0)
                {
                    geometry::Point point_to_goal = goal.position() - start.position();
                    units::Angle dir_to_goal = geometry::atan(point_to_goal);
                    units::Distance dx = distance_step * units::cos(dir_to_goal);
                    units::Distance dy = distance_step * units::sin(dir_to_goal);

                    units::Angle orient_init = dir_to_goal;

                    // Check if the goal is behind the start pose
                    if(backwards_motion && point_to_goal.dot(start.orientationUnitVector()) < 0)
                    {
                        orient_init = g2o::normalize_theta((orient_init + units::pi).to(units::rad)) * units::rad;
                    }

                    units::Distance distance_to_goal = point_to_goal.magnitude();
                    double num_steps_d = distance_to_goal() / distance_step();
                    unsigned int num_steps = static_cast<unsigned int>(num_steps_d);

                    if(max_velocity > 0)
                    {
                        timestep = distance_step / max_velocity;
                    }

                    for(unsigned int i = 1; i <= num_steps; i++)
                    {
                        if(i == num_steps && num_steps_d == static_cast<double>(num_steps))
                        {
                            break;
                        }

                        addPoseAndTimeDiff(Pose(start.x() + i * dx, start.y() + i * dy, orient_init), timestep);
                    }
                }

                // Manually insert more samples if  needed
                if(m_poses.size() < min_samples - 1)
                {
                    while(m_poses.size() < min_samples - 1)
                    {
                        Pose intermediate_pose = (m_poses.back()->pose() + goal) / 2.0;
                        if(max_velocity > 0)
                        {
                            timestep = intermediate_pose.position().distance(m_poses.back()->pose().position()) / max_velocity;
                        }

                        addPoseAndTimeDiff(intermediate_pose, timestep);
                    }
                }

                // Add goal
                if(max_velocity > 0)
                {
                    timestep = goal.position().distance(m_poses.back()->pose().position()) / max_velocity;
                }
                addPoseAndTimeDiff(goal, timestep);
                setPoseVertexFixed(m_poses.size() - 1, true);
            }
            else
            {
                // throw error
            }
        }

        void TimedElasticBand::updateAndPrune(const Pose* new_start, const Pose* new_goal, int min_samples)
        {
            // first and simple appraoch: change only start confs (amd virtual start confs)
            if(new_start && size() > 0)
            {
                // Find nearest state in order to prune the trajectory
                // (remove already passed states
                units::Distance distance_cache = new_start->position().distance(pose(0).position());
                units::Distance dist;
                int lookahead = std::min<int>(size() - min_samples, 10);

                int nearest_index = 0;
                for(int i = 1; i <= lookahead; i++)
                {
                    dist = new_start->position().distance(pose(i).position());
                    if( dist < distance_cache)
                    {
                        distance_cache = dist;
                        nearest_index = i;
                    }
                    else
                    {
                        break;
                    }
                }

                // prune trajectory at the beginning (and extrapolate sequences at the end
                // if the horizon is fixed)
                if(nearest_index > 0)
                {
                    removePoses(1, nearest_index);
                    removeTimeDiffs(1, nearest_index);
                }

                // update start
                m_poses[0]->setPose(*new_start);
            }

            if(new_goal && size()> 0)
            {
                m_poses.back()->setPose(*new_goal);
            }
        }

        void TimedElasticBand::autoResize(const units::Time& dt_ref, const units::Time& dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
        {
            bool modified = true;

            for(int rep = 0; rep < 100; rep++)
            {
                modified = false;
                for(int i = 0; i < sizeTD(); i++)
                {
                    if(timeDiff(i) > dt_ref + dt_hysteresis && sizeTD() < max_samples)
                    {
                        units::Time new_time = 0.5 * timeDiff(i);

                        m_time_diffs[i]->setDt(new_time);
                        insertPose(i + 1, (pose(i) + pose(i + 1)) / 2.0);

                        modified = true;
                    }
                    else if(timeDiff(i) < dt_ref - dt_hysteresis && sizeTD() > min_samples)
                    {
                        if(i < sizeTD() - 1)
                        {
                            m_time_diffs[i + 1]->setDt(timeDiff(i + 1) + timeDiff(i));
                            removeTimeDiff(i);
                            removePose(i + 1);
                        }

                        modified = true;
                    }
                }
                if(fast_mode)
                {
                    break;
                }
            }
        }

        void TimedElasticBand::setPoseVertexFixed(int index, bool fixed)
        {
            assert(index < m_poses.size());
            m_poses[index]->setFixed(fixed);
        }

        void TimedElasticBand::setTimeDiffVertexFixed(int index, bool fixed)
        {
            assert(index < m_time_diffs.size());
            m_time_diffs[index]->setFixed(fixed);
        }

        void TimedElasticBand::clear()
        {
            m_poses.clear();
            m_time_diffs.clear();
        }

        int TimedElasticBand::findClosestTrajectoryPose(const geometry::Point& ref, units::Distance* distance, int begin_index) const
        {
            units::Distance d = std::numeric_limits<double>::max();
            int n = -1;

            for(int i = begin_index; i < size(); i++)
            {
                units::Distance temp = ref.distance(m_poses[i]->position());
                if(temp < d)
                {
                    d = temp;
                    n = i;
                }
            }
            if(distance)
            {
                *distance = d;
            }
            return n;
        }

        int TimedElasticBand::findClosestTrajectoryPose(const geometry::Point& start, const geometry::Point& end, units::Distance* distance, int begin_index) const
        {
            units::Distance d = std::numeric_limits<double>::max();
            int n = -1;

            for(int i = begin_index; i < size(); i++)
            {
                units::Distance temp = geometry::utils::pointToSegment(m_poses[i]->position(), start, end);
                if(temp < d)
                {
                    d = temp;
                    n = i;
                }
            }
            if(distance)
            {
                *distance = d;
            }
            return n;
        }

        int TimedElasticBand::findClosestTrajectoryPose(const geometry::Polygon& polygon, units::Distance* distance, int begin_index) const
        {
            if(polygon.size() == 0)
            {
                return -1;
            }
            else if(polygon.size() == 1)
            {
                return findClosestTrajectoryPose(polygon.front(), distance);
            }
            else if(polygon.size() == 2)
            {
                return findClosestTrajectoryPose(polygon.front(), polygon.back(), distance);
            }

            units::Distance d = std::numeric_limits<double>::max();
            int n = -1;

            for(int i = begin_index; i < size(); i++)
            {
                geometry::Point point = m_poses[i]->pose().position();
                units::Distance temp = std::numeric_limits<double>::max();
                for(int j = 0; j < polygon.size() - 1; j++)
                {
                    temp = units::min(temp, geometry::utils::pointToSegment(point, polygon[j], polygon[j + 1]));
                }
                temp = units::min(temp, geometry::utils::pointToSegment(point, polygon.back(), polygon.front()));
                if(temp < d)
                {
                    d = temp;
                    n = i;
                }
            }
            if(distance)
            {
                *distance = d;
            }
            return n;
        }

        int TimedElasticBand::findClosestTrajectoryPose(std::shared_ptr<Obstacle> obstacle, units::Distance* distance) const
        {
            std::shared_ptr<PointObstacle> p_obstacle = std::dynamic_pointer_cast<PointObstacle>(obstacle);
            if(p_obstacle)
            {
                return findClosestTrajectoryPose(p_obstacle->centroid(), distance);
            }

            std::shared_ptr<LineObstacle> l_obstacle = std::dynamic_pointer_cast<LineObstacle>(obstacle);
            if(l_obstacle)
            {
                return findClosestTrajectoryPose(l_obstacle->start(), l_obstacle->end(), distance);
            }

            std::shared_ptr<PolygonObstacle> poly_obstacle = std::dynamic_pointer_cast<PolygonObstacle>(obstacle);
            if(poly_obstacle)
            {
                return findClosestTrajectoryPose(poly_obstacle->polygon(), distance);
            }

            return findClosestTrajectoryPose(obstacle->centroid(), distance);
        }

        size_t TimedElasticBand::size() const
        {
            return m_poses.size();
        }

        size_t TimedElasticBand::sizeTD() const
        {
            return m_time_diffs.size();
        }

        units::Time TimedElasticBand::sumAllTimeDiffs() const
        {
            units::Time time = 0;
            for(std::shared_ptr<VertexTimeDiff> dt : m_time_diffs)
            {
                time += dt->dt();
            }
            return time;
        }

        units::Time TimedElasticBand::sumAllTimeDiffsUpToIndex(int index) const
        {
            units::Time time = 0;
            for(int i = 0; i < index; i++)
            {
                time += m_time_diffs[i]->dt();
            }
            return time;
        }

        units::Distance TimedElasticBand::accumulatedDistance() const
        {
            units::Distance dist = 0;
            for(int i = 1; i < size(); i++)
            {
                dist += m_poses[i]->pose().position().distance(m_poses[i - 1]->pose().position());
            }
            return dist;
        }

        bool TimedElasticBand::detectDetoursBackwards(const units::Angle& threshold) const
        {
            if(size() < 2)
            {
                return false;
            }

            geometry::Point d_start_goal = m_poses.back()->pose().position() - m_poses.front()->pose().position();
            d_start_goal = d_start_goal.normalize();

            for(int i = 0; i < size(); i++)
            {
                geometry::Point orient_vector( units::cos(m_poses[i]->theta()), units::sin(m_poses[i]->theta()) );
                if(orient_vector.dot(d_start_goal) < threshold)
                {
                    return true;
                }
            }

            return false;
        }
    }
}
