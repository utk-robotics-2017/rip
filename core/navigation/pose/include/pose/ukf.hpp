#ifndef UKF_HPP
#define UKF_HPP

#include "pose/filter_base.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            /**
             * Unscented Kalman Filter
             *
             * Implementation of an Unscented Kalman Filter (UKF). This
             * class derives from FilterBase and overrides the predict()
             * and correct() methods in keeping with the discrete time
             * UKF algorithm. The algorithm was derived from the UKF
             * Wikipedia article at
             * (http://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter)
             * ...and this paper:
             * J. J. LaViola, Jr., “A comparison of unscented and extended Kalman
             * filtering for estimating quaternion motion,” in Proc. American Control
             * Conf., Denver, CO, June 4–6, 2003, pp. 2435–2440
             * Obtained here: http://www.cs.ucf.edu/~jjl/pubs/laviola_acc2003.pdf
             */
            class Ukf : public FilterBase
            {
            public:
                /**
                 * Constructor
                 *
                 * @param args - Generic argument container. It is assumed
                 * that args[0] constains the alpha parameter, args[1] contains
                 * the kappa parameter, and args[2] contains the beta parameter.
                 */
                explicit Ukf(std::vector<double> args);

                /**
                 * Destructor
                 */
                ~Ukf();

                /**
                 * Carries out the correct step in the predict/update cycle
                 *
                 * @param measurement - The measurement to fuse with our estimate
                 */
                void correct(const Measurement& measurement) override;

                /**
                 * Carries out the predict step in the predict/update cycle
                 *
                 * Projects the state and error matrices forward using a model of
                 * the vehicle's motion.
                 *
                 * @param reference_time - The time at which the prediction is being made
                 * @param delta - The time step over which to predict
                 */
                void predict(const double reference_time, const double delta);

            protected:
                std::vector<Eigen::VectorXd> m_sigma_points; //!< Used to sample possible next states during prediction.
                Eigen::MatrixXd m_weighted_covar_sqrt; //!< This matrix is used to generate the sigma points
                std::vector<double> m_state_weights; //!< The weights associated with each sigma point when generating a new state
                std::vector<double> m_covar_weights; //!< The weights assocated with each sigma point when calculating a predicted covariance error
                double m_lambda; //!< used in weight generation for sigma points
                bool m_uncorrected; //!< Used to determine if we need to re-compute the sigma points when carrying out multiple corrections
            };
        }
    }
}

#endif //UKF_HPP
