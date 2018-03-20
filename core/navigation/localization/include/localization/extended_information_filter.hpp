#ifndef EXTENDED_INFORMATION_FILTER_HPP
#define EXTENDED_INFORMATION_FILTER_HPP

#include "extended_kalman_filter.hpp"
#include "probability/continuous/gaussian_canonical.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template <typename X, typename U, typename Z>
            class ExtendedInformationFilter
            {
                using probability::Matrix;
                using probability::Vector;
                using probability::GaussianCanonical;
            public:
                ExtendedInformationFilter() {}
                virtual ~ExtendedInformationFilter() {}

                std::shared_ptr< GaussianCanonical<X> > marginalize(
                            const ExtendedKalmanActionModel<U, X>& model,
                            const U& action,
                            const GaussianCanonical<X>& belief)
                {


                    const X& mu = belief.getMu();
                    const Matrix& Sigma = belief.inversePrecision();

                    Matrix G = model.getStateJacobian(action, mu);
                    Matrix Gt = G.transpose();
                    Matrix R = model.getError(action, mu).inverse();
                    Matrix nextOmega = (G * Sigma * Gt + R).inverse();

                    X nextXi;
                    nextXi.set(nextOmega * model.getMean(action, mu));

                    return std::make_shared<GaussianCanonical<X>>(nextXi, nextOmega);
                }

                std::shared_ptr<probability::continuous::GaussianCanonical<X>>
                        bayesianInference(
                            const ExtendedKalmanSensorModel<Z, X>& model,
                            const Z& observation,
                            const probability::continuous::GaussianCanonical<X>& belief)
                {
                    using probability::Matrix;
                    using probability::Vector;
                    using probability::continuous::GaussianCanonical;

                    const X& mu = belief.getMu();
                    const Matrix& Omega = belief.precision();

                    Matrix H = model.getJacobian(mu);
                    Matrix Ht = H.transpose();
                    Matrix Q = model.getError(observation);
                    Matrix QInv = Q.inverse();
                    Matrix nextOmega = Omega + Ht * QInv * H;

                    const Vector& xi = belief.information();
                    Z expectedObservation = model.getMean(mu);

                    Vector nextXi = xi + Ht * QInv * (observation - expectedObservation + H * mu);

                    return std::make_shared<GaussianCanonical<X>>(nextXi, nextOmega);
                }
            };

        }
    }
}

#endif // EXTENDED_INFORMATION_FILTER_HPP
