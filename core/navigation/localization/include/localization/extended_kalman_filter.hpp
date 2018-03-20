#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "probability/data/matrix.hpp"
#include "probability/continuous/gaussian_moment.hpp"


namespace rip
{
    namespace navigation
    {
        namespace localization
        {

            using Matrix = probability::Matrix;

            template<typename U, typename X>
            class ExtendedKalmanActionModel : public GaussianActionModel<U, X>
            {
            public:
                ExtendedKalmanActionModel() {}
                virtual ~ExtendedKalmanActionModel() {}

                virtual Matrix GetStateJacobian(const U& action, const X& state) const = 0;
                virtual Matrix GetActionJacobian(const U& action, const X& state) const = 0;
            };

            template<typename Z, typename X>
            class ExtendedKalmanSensorModel : public GaussianSensorModel<Z, X>
            {
            public:
                ExtendedKalmanSensorModel() {}
                virtual ~ExtendedKalmanSensorModel() {}

                virtual Matrix GetJacobian(const X& x) const = 0;
            };

            template<typename X, typename U, typename Z>
            class ExtendedKalmanFilter
            {
                using probability::GaussianMoment;
                using probability::Vector;
            public:
                ExtendedKalmanFilter() {}
                virtual ~ExtendedKalmanFilter() {}

                std::shared_ptr< GaussianMoment<X> > marginalize(
                                                    const ExtendedKalmanActionModel<U, X>& model,
                                                    const U& action,
                                                    const GaussianMoment<X>& belief)
                {

                    X x0 = belief.mean();
                    Matrix e0 = belief.covariance();

                    // time update
                    Matrix g = model.getStateJacobian(action, x0);
                    Matrix gt = g.transpose();
                    Matrix r = model.getError(action, x0);

                    X x1 = model.getMean(action, x0);
                    Matrix e1 = g * e0 * gt + r;

                    return std::make_shared<GaussianMoment<X>>(x1, e1);
                }

                std::shared_ptr<GaussianMoment<X>> marginalize(
                                                    const ExtendedKalmanActionModel<U, X>& model,
                                                    const GaussianMoment<U>& action,
                                                    const GaussianMoment<X>& belief)
                {

                    X x0 = belief.mean();
                    U u = action.mean();
                    Matrix e0 = belief.covariance();
                    Matrix eu = action.covariance();

                    // time update
                    Matrix gx = model.getStateJacobian(u, x0);
                    Matrix gxt = gx.transpose();
                    Matrix gu = model.getActionJacobian(action.Mean, x0);
                    Matrix gut = gu.transpose();
                    Matrix r = model.getError(u, x0);

                    X x1 = model.getMean(u, x0);
                    Matrix e1 = gx * e0 * gxt + gu * eu * gut + r;

                    return std::make_shared<GaussianMoment<X>>(x1, e1);
                }


                std::shared_ptr<GaussianMoment<X>> bayesianInference(
                                                    const ExtendedKalmanSensorModel<Z, X>& sensor_model,
                                                    const Z& observation,
                                                    const GaussianMoment<X>& belief)
                {

                    // Table 3.3 p59
                    X x0 = belief.mean();
                    Matrix e0 = belief.covariance();

                    Z expected_z = sensor_model.getMean(x0);
                    Matrix q = sensor_model.getError(observation);

                    Matrix h = sensor_model.getJacobian(x0);
                    Matrix ht = h.transpose();

                    Matrix i = Matrix::identity(e0.order());

                    Matrix e0_ht = e0 * ht;
                    Matrix k = e0_ht * (h * e0_ht + q).inverse();
                    Vector v = (x0 + k * (observation - expected_z));
                    X x1;
                    x1.set(std::move(v));
                    Matrix e1 = (i - k * h) * e0;

                    return std::make_shared<GaussianMoment<X>>(x1, e1);
                }
            };
        }
    }
}


#endif // EXTENDED_KALMAN_FILTER_HPP
