#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "matrix.hpp"
#include "vector.hpp"
#include "gaussian_moment.hpp"
#include "gaussian_action_model.hpp"
#include "gaussian_sensor_model.hpp"

#include <memory>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename U, typename X>
            class ExtendedKalmanActionModel : public GaussianActionModel<U, X>
            {
            public:
                ExtendedKalmanActionModel()
                {
                }

                virtual ~ExtendedKalmanActionModel()
                {
                }

                virtual Matrix getStateJacobian(const U& action, const X& state) const = 0;

                virtual Matrix getActionJacobian(const U& action, const X& state) const = 0;
            };

            template<typename Z, typename X>
            class ExtendedKalmanSensorModel : public GaussianSensorModel<Z, X>
            {
            public:
                ExtendedKalmanSensorModel()
                {
                }

                virtual ~ExtendedKalmanSensorModel()
                {
                }

                virtual Matrix getJacobian(const X& x) const = 0;
            };

            template<typename X, typename U, typename Z>
            class ExtendedKalmanFilter
            {
            public:
                ExtendedKalmanFilter()
                {
                }

                virtual ~ExtendedKalmanFilter()
                {
                }

                std::shared_ptr <GaussianMoment<X>>
                marginalize(const ExtendedKalmanActionModel<U, X>& model, const U& action,
                            const GaussianMoment <X>& belief)
                {

                    X x0 = belief.mean();
                    Matrix e0 = belief.covariance();

                    // time update
                    Matrix g = model.GetStateJacobian(action, x0);
                    Matrix gt = g.Transpose();
                    Matrix r = model.GetError(action, x0);

                    X x1 = model.GetMean(action, x0);
                    Matrix e1 = g * e0 * gt + r;

                    return std::make_shared < GaussianMoment < X >> (x1, e1);
                }

                std::shared_ptr <GaussianMoment<X>>
                marginalize(const ExtendedKalmanActionModel<U, X>& model, const GaussianMoment <U>& action,
                            const GaussianMoment <X>& belief)
                {

                    X x0 = belief.mean();
                    U u = action.mean();
                    Matrix e0 = belief.covariance();
                    Matrix eu = action.covariance();

                    // time update
                    Matrix gx = model.GetStateJacobian(u, x0);
                    Matrix gxt = gx.Transpose();
                    Matrix gu = model.GetActionJacobian(action.Mean, x0);
                    Matrix gut = gu.Transpose();
                    Matrix r = model.GetError(u, x0);

                    X x1 = model.GetMean(u, x0);
                    Matrix e1 = gx * e0 * gxt + gu * eu * gut + r;

                    return std::make_shared < GaussianMoment < X >> (x1, e1);
                }


                std::shared_ptr <GaussianMoment<X>>
                bayesianInference(const ExtendedKalmanSensorModel<Z, X>& sensor_model, const Z& observation,
                                  const GaussianMoment <X>& belief)
                {

                    // Table 3.3 p59
                    X x0 = belief.mean();
                    Matrix e0 = belief.covariance();

                    Z expected_z = sensor_model.GetMean(x0);
                    Matrix q = sensor_model.GetError(observation);

                    Matrix h = sensor_model.GetJacobian(x0);
                    Matrix ht = h.Transpose();

                    Matrix i = Matrix::Identity(e0.order());

                    Matrix e0_ht = e0 * ht;
                    Matrix k = e0_ht * (h * e0_ht + q).Inverse();
                    Vector v = (x0 + k * (observation - expected_z));
                    X x1;
                    x1.Set(std::move(v));
                    Matrix e1 = (i - k * h) * e0;

                    return std::make_shared < GaussianMoment < X >> (x1, e1);
                }
            };
        }
    }
}

#endif //EXTENDED_KALMAN_FILTER_HPP