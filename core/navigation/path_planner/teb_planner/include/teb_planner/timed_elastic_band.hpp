#ifndef TIMED_ELASTIC_BAND_HPP
#define TIMED_ELASTIC_BAND_HPP

#include <vector>

#include <eigen3/Eigen/Core>

#include <units/units.hpp>
#include <geometry/polygon.hpp>

#include "pose.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"
#include "obstacle.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * All trajectory related methods (initialization, modifying, etc) are
         * implemented inside this class.
         * Let \f$ Q = \lbrace \mathbf{s}_i \rbrace_{i=0...n},\ n \in \mathbb{N} \f$
         * be a sequence of poses,
         * in which \f$ \mathbf{s}_i = [x_i, y_i, \beta_i]^T \in \mathbb{R}^2 \times S^1 \f$
         * denotes a single pose of the robot.
         * The Timed Elastic Band (TEB) augments this sequence of poses by
         * incorporating time intervals between two consecutive poses, resulting
         * in a sequence of \c n-1 time intervals \f$ \Delta T_i \f$:
         * \f$ \tau = \lbrace \Delta T_i \rbrace_{i=0...n-1} \f$.
         * Each time interval denotes the time that the robot requires to transition
         * from the current configuration to the next one.
         * The tuple of both sequences defines the underlying trajectory.
         */
        class TimedElasticBand
        {
        public:
            /**
             * Constructor
             */
            TimedElasticBand() = default;

            /**
             * Destructor
             */
            ~TimedElasticBand();

            bool isInit() const;

            /**
             * Returns the list of poses
             */
            std::vector< std::shared_ptr<VertexPose> > poses() const;

            /**
             * Returns the specified pose
             */
            Pose pose(int index) const;

            /**
             * Returns the list of time diffs
             */
            std::vector< std::shared_ptr<VertexTimeDiff> > timeDiffs() const;

            /**
             * Returns the specified time diff
             */
            units::Time timeDiff(int index) const;

            /**
             * Returns the specified vertex of the pose
             */
            std::shared_ptr<VertexPose> poseVertex(int index) const;

            /**
             * Returns the specified vertex of the time diff
             */
            std::shared_ptr<VertexTimeDiff> timeDiffVertex(int index) const;

            /**
             * Append a new pose to the sequence
             */
            void addPose(const Pose& pose, bool fixed=false);

            /**
             * Append a new time diff to the sequence
             */
            void addTimeDiff(const units::Time& dt, bool fixed=false);

            void addPoseAndTimeDiff(const Pose& pose, const units::Time& dt);

            /**
             * Insert a new pose at the specified position
             */
            void insertPose(int index, const Pose& pose);

            /**
             * Insert a new time diff at the specified position
             */
            void insertTimeDiff(int index, const units::Time& dt);

            /**
             * Remove the pose at the specified position
             */
            void removePose(int index);

            /**
             * Removes the @p number of poses starting from the
             * specified @p index
             */
            void removePoses(int index, int number);

            /**
             * Remove the time diff at the specified position
             */
            void removeTimeDiff(int index);

            /**
             * Removes the @p number of time diffs starting from the
             * specified @p index
             */
            void removeTimeDiffs(int index, int number);

            /**
             * Initialize a trajectory between a given start and goal pose
             *
             * The implemented algorithm subsamples the straight line between
             * start and goal using a given discretization width.
             *
             * @param start The starting pose of the trajectory
             * @param goal The goal pose of the trajectory
             * @param distance_step The euclidean distance between two consecutive poses
             * @param max_velocity Maximum translational velocity
             * @param min_samples Minimum number of samples in the trajectory
             */
            void initTrajectoryToGoal(const Pose& start, const Pose& goal, const units::Distance& distance_step, const units::Velocity& max_velocity, int mimimum_samples, bool backwards_motion = false);

            /**
             * Update the start and goal poses for an existing trajectory
             * @param new_start       New start pose (optional)
             * @param new_goal        New goal pose (optional)
             * @param minimum_samples Minimum number of samples in the trajectory
             */
            void updateAndPrune(const Pose* new_start = nullptr, const Pose* new_goal = nullptr, int min_samples = 3);

            /**
             * Resize the trajectory by removing or inserting a (pose, dt) pait depending on a reference temporal resolution.
             *
             * @param dt_ref Reference temporal resolution
             * @param dt_hysteresis Hysteresis to avoid oscillations
             * @param min_samples Minimum number of samples that should remain in the trajectory after resizing
             * @param max_samples Maximum number of samples that should not be exceeded during resizing
             * @param fast_mode If true, the trajectory is iterated once to insert or erase points; if false, the trajectory
             *                  is repeated iterated until no poses are added or removed anymore
             */
            void autoResize(const units::Time& dt_ref, const units::Time& dt_hysteresis, int min_samples = 3, int max_samples = 1000, bool fast_mode = false);

            /**
             * Sets the pose vertex at the specified @p index to be fixed
             * or unfixed during optimization
             */
            void setPoseVertexFixed(int index, bool fixed);

            /**
             * Sets the time diff vertex at the specified @p index to be fixed
             * or unfixed during optimization
             */
            void setTimeDiffVertexFixed(int index, bool fixed);

            /**
             * Clear all the poses and time diffs
             */
            void clear();

            /**
             * Find the closest point on the trajectory w.r.t. to a provided reference point
             */
            int findClosestTrajectoryPose(const geometry::Point& ref, units::Distance* distance = nullptr, int begin_index=0) const;

            /**
             * Find the closest point on the trajectory w.r.t. to a provided line segment
             */
            int findClosestTrajectoryPose(const geometry::Point& start, const geometry::Point& end, units::Distance* distance = nullptr, int begin_index=0) const;

            /**
             * Find the closest point on the trajectory w.r.t. to a provided polygon
             */
            int findClosestTrajectoryPose(const geometry::Polygon& polygon, units::Distance* distance = nullptr, int begin_index=0) const;

            /**
             * Find the closest point on the trajectory w.r.t. to a provided obstacle
             */
            int findClosestTrajectoryPose(std::shared_ptr<Obstacle> obstacle, units::Distance* distance) const;

            /**
             * Returns the number of poses
             */
            size_t size() const;

            /**
             * Returns the number of time diffs
             */
            size_t sizeTD() const;

            /**
             * Returns the total time difference
             */
            units::Time sumAllTimeDiffs() const;

            /**
             * Returns the total time difference up to the specified @p index
             */
            units::Time sumAllTimeDiffsUpToIndex(int index) const;

            /**
             * Returns the total distance of the trajectory
             */
            units::Distance accumulatedDistance() const;

            /**
              * Detect whether the trajectory contains detours.
              *
              * 1. The trajectory contains parts that increase the distance to the goal.
              * 2. The trajectory consist of backwards motions at the begining of the trajectory.
              */
            bool detectDetoursBackwards(const units::Angle& threshold = 0) const;

        private:
            std::vector< std::shared_ptr<VertexPose> > m_poses;
            std::vector< std::shared_ptr<VertexTimeDiff> > m_time_diffs;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // TIMED_ELASTIC_BAND_HPP
