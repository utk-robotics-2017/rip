#ifndef TIMED_ELASTIC_BAND_HPP
#define TIMED_ELASTIC_BAND_HPP

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
            TimedElasticBand();

            std::vector<Pose> poses() const;

            std::vector<TimeDiff> timeDiffs() const;

            units::Time timeDiff(int index);

            Pose pose(int index) const;

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
            void initTrajectoryToGoal(const Pose& start, const Pose& goal, const units::Distance& distance_step, const units::Velocity& max_velocity, int mimimum_samples)
            {

            }

            /**
             * Update the start and goal poses for an existing trajectory
             * @param new_start       New start pose (optional)
             * @param new_goal        New goal pose (optional)
             * @param minimum_samples Minimum number of samples in the trajectory
             */
            void updateAndPrune(Pose new_start, Pose new_goal, int minimum_samples);

        private:
        };
    }
}

#endif // TIMED_ELASTIC_BAND_HPP
