#ifndef EKF_HPP
#define EKF_HPP

#include "pose/filter_base.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            /**
             * Extend Kalman Filter class
             *
             * Implementation of an Extended Kalman Filter (EKF). This
             * class derives from FilterBase and overrides the predict()
             * and correct() methods in keeping with the discrete time
             * EKF algorithm.
             */
            class Ekf : public FilterBase
            {
            public:
                /**
                 * Constructor
                 *
                 * @param args - generic argument container
                 */
                explicit Ekf(std::vector<double> args = std::vector<double>());

                /**
                 * Destructor
                 */
                ~Ekf();

                /**
                 * Carries out the correct step in the predict/update cycle.
                 *
                 * @param measurement - The measurement to fuse with our estimate
                 */
                void correct(const Measurement& measurement) override;

                /**
                 * Carries out the predict step in the predict/update cycle
                 *
                 * Projects the state and error matrices forward using a model
                 * of the vehicle's motion.
                 *
                 * @param reference_time - The time at which the prediction is being made
                 * @param delta - The time step over which to predict
                 */
                void predict(const double reference_time, const double delta);
            };
        }
    }
}

#endif //EKF_HPP
