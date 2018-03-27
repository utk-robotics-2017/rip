#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

#include <vector>
#include <memory>

#include "pose/gaussian_moment.hpp"
#include "pose/matrix.hpp"
#include "pose/vector.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename X2, typename Y>
            struct SigmaPoint
            {
                static std::vector<SigmaPoint<X2, Y>>
                createSigmaPoints(const GaussianMoment <X2>& belief, double alpha, double beta, double lambda)
                {

                    std::size_t order = belief.order();
                    double n = static_cast<double>(order);

                    Matrix p0 = ((n + lambda) * belief.covariance());
                    // we need to ensure that P0 is symmetric
                    Matrix p1 = p0.roundSymmetry();
                    Matrix p = p1.sqrt().Transpose();

                    std::vector<SigmaPoint<X2, Y> > sigma_pts(2 * order + 1);

                    double wm0 = lambda / (lambda + n);
                    double wc0 = wm0 + (1 - (alpha * alpha) + beta);
                    double wn = 1 / (2 * (lambda + n));

                    sigma_pts[0] = SigmaPoint<X2, Y>(belief.mean(), wm0, wc0);
                    for(std::size_t i = 0; i < order; i ++)
                    {
                        X2 x = belief.mean();
                        Vector dx = p.column(i);
                        X2 x1;
                        x1.Set(x + dx);
                        X2 x2;
                        x2.Set(x - dx);
                        sigma_pts[i + 1] = SigmaPoint<X2, Y>(x1, wn, wn);
                        sigma_pts[i + 1 + order] = SigmaPoint<X2, Y>(x2, wn, wn);
                    }
                    return sigma_pts;
                }

                static Y getWeightedMean(const std::vector<SigmaPoint<X2, Y> >& sigma_pts, std::size_t n)
                {
                    std::vector<double> a(n);
                    for(std::size_t i = 0; i < sigma_pts.size(); i ++)
                    {
                        SigmaPoint<X2, Y> sp = sigma_pts[i];
                        for(std::size_t j = 0; j < a.size(); j ++)
                        {
                            a[j] += sp.mean_weight * sp.post[j];
                        }
                    }
                    Vector y(std::move(a));
                    return Y(std::move(y));
                }

                static Matrix getWeightedError(const std::vector<SigmaPoint<X2, Y> >& sigma_pts, std::size_t n, Y next)
                {

                    Matrix next_e(n, n);
                    for(std::size_t i = 0; i < sigma_pts.size(); i ++)
                    {
                        SigmaPoint<X2, Y> sp = sigma_pts[i];
                        Vector dt = sp.post - next;
                        next_e = next_e + (sp.error_weight * dt.Cross(dt) + sp.error);
                    }
                    return next_e;
                }

                X2 prior;
                Y post;
                Matrix error;
                double mean_weight;
                double error_weight;

                SigmaPoint(X2 p, double w1, double w2) : prior(p), mean_weight(w1), error_weight(w2)
                {
                }

                SigmaPoint() : prior(), mean_weight(0.0), error_weight(0.0)
                {
                }
            };

            template<typename X, typename U, typename Z>
            class UnscentedKalmanFilter
            {
            public:
                UnscentedKalmanFilter(double alpha, double beta, double kappa) : m_alpha(alpha), m_beta(beta), m_kappa(kappa)
                {
                }

                virtual ~UnscentedKalmanFilter()
                {
                }

                std::shared_ptr< GaussianMoment<X> > marginalize(const GaussianActionModel <U, X>& model, const U& action, const GaussianMoment <X>& belief)
                {
                    std::vector<SigmaPoint<X, X>> sigma_x = SigmaPoint<X, X>::createSigmaPoints(belief, m_alpha, m_beta,
                                                                                                lambda(belief));

                    for(std::size_t i = 0; i < sigma_x.size(); i ++)
                    {
                        sigma_x[i].post = model.getMean(action, sigma_x[i].prior);
                        sigma_x[i].error = model.getError(action, belief.mean());
                    }

                    std::size_t n = belief.order();
                    X next_x = SigmaPoint<X, X>::getWeightedMean(sigma_x, n);
                    Matrix next_e = SigmaPoint<X, X>::getWeightedError(sigma_x, n, next_x);

                    return std::make_shared<GaussianMoment < X> > (next_x, next_e);
                }

                std::shared_ptr<GaussianMoment < X>>bayesianInference(const GaussianSensorModel <Z, X>& model, const Z& observation, const GaussianMoment <X>& belief)
                {
                    std::vector<SigmaPoint<X, Z>> sigma_z = SigmaPoint<X, Z>::createSigmaPoints(belief, m_alpha, m_beta,
                                                                                                lambda(belief));

                    for(std::size_t i = 0; i < sigma_z.size(); i ++)
                    {
                        sigma_z[i].post = model.getMean(sigma_z[i].prior);
                        sigma_z[i].error = model.getError(sigma_z[i].post);
                    }

                    std::size_t n = belief.order();
                    Z next_z = SigmaPoint<X, Z>::getWeightedMean(sigma_z, n);
                    Matrix S = SigmaPoint<X, Z>::getWeightedError(sigma_z, n, next_z);

                    Matrix E(n, n);
                    for(std::size_t i = 0; i < sigma_z.size(); i ++)
                    {
                        SigmaPoint<X, Z> sp = sigma_z[i];
                        Vector dx = sigma_z[i].prior - belief.mean();
                        Vector du = sigma_z[i].post - next_z;
                        E = E + sp.error_weight * dx.cross(du);
                    }

                    Matrix K = E * S.inverse();

                    X next_x(belief.mean() + K * (observation - next_z));
                    Matrix next_e = belief.covariance() - K * S * K.transpose();

                    return std::make_shared<GaussianMoment < X> > (next_x, next_e);
                }

            private:
                double lambda(const GaussianMoment <X>& belief) const
                {
                    double n = static_cast<double>(belief.order());
                    return (m_alpha * m_alpha) * (n + m_kappa) - n;
                }

                /**
                 * Primary scaling factor and determines the spread of the Sigma points
                 * around the x mean and is usually set to a small positive value (e.g.1e-3)
                 */
                double m_alpha;

                /**
                 * Beta is the secondary scaling factor and determines the weight given to
                 * the mean when recombing the sigma point.
                 * Beta incorporates additional higher order knowledge about the
                 * distribution Beta=2 indicates the distribution is Gaussian.
                 */
                double m_beta;

                /**
                 * Kappa is a tertiary scaling parameter and is usually set to zero
                 */
                double m_kappa;
            };
        }
    }
}

#endif //UNSCENTED_KALMAN_FILTER_HPP
