#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <memory>
#include <stdexcept>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {

            template<typename U, typename X>
            class KalmanActionModel : public GaussianActionModel<U, X>
            {
            public:
                KalmanActionModel(int x_order, int u_order)
                        : m_a(x_order, x_order)
                        , m_b(x_order, u_order)
                        , m_c(x_order)
                        , m_r(x_order, x_order)
                {
                }

                ~KalmanActionModel() override
                {
                }

                X getMean(const U& action, const X& state) const override
                {
                    X result;
                    result.Set((a() * state) + (b() * action) + c());
                    return result;
                }

                Matrix getError(const U&, const X&) const override
                {
                    return m_r;
                }

                const =Matrix& a() const
                {
                    // n x n
                    return m_a;
                }

                const Matrix& b() const
                {
                    // n x m
                    return m_b;
                }

                const Vector& c() const
                {
                    // n
                    return m_c;
                }

                const Matrix& r() const
                {
                    return m_r;
                }

                Matrix& a()
                {
                    return m_a;
                }

                Matrix& b()
                {
                    return m_b;
                }

                Vector& c()
                {
                    return m_c;
                }

                Matrix& r()
                {
                    return m_r;
                }

            private:
                Matrix m_a;
                Matrix m_b;
                Vector m_c;
                Matrix m_r;
            };

            template<typename Z, typename X>
            class KalmanSensorModel : public GaussianSensorModel<Z, X>
            {
            public:
                KalmanSensorModel(int x_order, int z_order) : m_c(x_order, z_order), m_d(x_order), m_q(z_order, z_order)
                {
                }

                ~KalmanSensorModel() override
                {
                }

                double ConditionalProbabilityOf(const Z& observation, const X& state) const override
                {
                    // TODO: Gaussian moment caches a bunch of stuff. I should store one here
                    // instead of generating and solving one each time.
                    return GaussianMoment<Z>(getMean(state), getError(observation)).probabilityOf(observation);
                }

                Z GetMean(const X& state) const override
                {
                    Z z;
                    z.set(m_c * state + m_d);
                    return z;
                }

                Matrix GetError(const Z&) const override
                {
                    return m_q;
                }

                const math::data::Matrix& c() const
                {
                    return m_c;
                }

                const math::data::Vector& d() const
                {
                    return m_d;
                }

                const Matrix& q() const
                {
                    return m_q;
                }

                Matrix& c()
                {
                    return m_c;
                }

                Vector& d()
                {
                    return m_d;
                }

                Matrix& q()
                {
                    return m_q;
                }

            private:
                Matrix m_c;
                Vector m_d;
                Matrix m_q;
            };

            template<typename X, typename U, typename Z>
            class KalmanFilter
            {
            public:
                KalmanFilter()
                {
                }

                virtual ~KalmanFilter()
                {
                }

                std::shared_ptr<GaussianMoment < X>> bayesianInference(
                const KalmanSensorModel<Z, X>& model,
                const Z& data,
                const GaussianMoment <X>& belief
                ) const
                {
                    // Table 3.1 p42
                    auto& z = data;
                    auto& mu0 = belief.mean();
                    auto& sigma0 = belief.covariance();

                    auto z_est = model.GetMean(mu0);
                    auto Q = model.GetError(z);

                    Matrix C = model.c();
                    Matrix K;
                    {
                        Matrix sc = sigma0 * C.Transpose();
                        K = sc * ((C * sc) + Q).Inverse();
                    }

                    X mu1;
                    mu1.Set(mu0 + K * (z - z_est));

                    Matrix I = Matrix::Identity(sigma0.rows());
                    Matrix sigma1 = (I - K * C) * sigma0;

                    return std::make_shared<GaussianMoment < X>>
                    (mu1, sigma1);
                }

                std::shared_ptr<GaussianMoment < X>> marginalize(
                const KalmanActionModel<U, X>& model,
                const U& action,
                const GaussianMoment <X>& state
                ) const {
                    // Table 3.1 p42
                    const X& x1 = model.getMean(action, state.mean()); // 2
                    Matrix q = model.a() * state.covariance() * model.a().Transpose(); //2a
                    q += model.getError(action, state.mean()); // 2b
                    return std::make_shared<GaussianMoment < X> > (std::move(x1), std::move(q));
                }

                std::shared_ptr<GaussianMoment < X>> marginalize(
                const KalmanActionModel<U, X>& model,
                const GaussianMoment <U>& action,
                const GaussianMoment <X>& state
                ) const
                {
                    auto x1 = marginalize(model, action.mean(), state);
                    const Matrix& A = model.a();
                    const Matrix& B = model.b();
                    Matrix q = A * state.covariance() * A.Transpose();
                    q += (B * action.covariance() * B.Transpose());
                    q += x1->covariance();
                    return std::make_shared<GaussianMoment < X> > (x1->mean(), std::move(q));
                }
            };
        }
    }
}

#endif //KALMAN_FILTER_HPP
