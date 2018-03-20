#ifndef INFORMATION_FILTER_HPP
#define INFORMATION_FILTER_HPP

#include "probability/continuous/gaussian_canonical.hpp"
#include "probability/data/matrix.hpp"
#include "probability/data/vector.hpp"
#include "kalman_filter.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template<typename X, typename U, typename Z>
            class InformationFilter
            {
                using probability::GaussianCanonical;
                using probability::Matrix;
                using probability::Vector;
            public:
                InformationFilter() {}
                virtual ~InformationFilter() {}

                std::shared_ptr<GaussianCanonical<X>> marginalize(
                        const KalmanActionModel<U, X>& model, const U& action,
                        const GaussianCanonical<X>& belief)
                {
                    const Vector& xi = belief.information();
                    const Matrix& A = model.a();
                    const Matrix& B = model.b();
                    const Vector& C = model.c();
                    const Matrix& R = model.r();
                    Matrix At = A.transpose();
                    Matrix Sigma = belief.inversePrecision();
                    Matrix ASigma = A * Sigma;

                    Matrix nextOmega = (ASigma * At + R).inverse();
                    X nextXi;
                    nextXi.set(nextOmega * (ASigma * xi + B * action + C));

                    return std::make_shared<GaussianCanonical<X>>(std::move(nextXi), std::move(nextOmega));
                }

                std::shared_ptr<GaussianCanonical<X>> bayesianInference(
                        const KalmanSensorModel<Z, X>& model,
                        const Z& data,
                        const GaussianCanonical<X>& belief)
                {

                    const Vector& Xi = belief.information();
                    const Matrix& Omega = belief.precision();
                    const Matrix& C = model.c();
                    Matrix Ct = C.transpose();
                    const Matrix Q = model.q();
                    const Matrix QInv = Q.inverse();
                    const Matrix CtQInv = Ct * QInv;

                    Matrix nextOmega = CtQInv * C + Omega;
                    Vector nextXi = CtQInv * data + Xi;

                    return std::make_shared<GaussianCanonical<X>>(std::move(nextXi), std::move(nextOmega));
                }
            };

        }
    }
}

#endif // INFORMATION_FILTER_HPP