#include <gtest/gtest.h>

#include "localization/probability/random_conditional.hpp"
#include "localization/probability/ranged_uniform.hpp"
#include "localization/probability/bayes_distribution.hpp"
#include "localization/sensor_model.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                namespace test
                {
                    DEFINE_VECTOR1(X);
                    DEFINE_VECTOR1(Z);

                    /**
                     * A sensor model that says that X is within 0.25 units observed Z
                     */
                    class SensorModel : public RandomConditional<Z, X>
                    {
                    public:
                        double conditionalProbabilityOf(const Z& z, const X& x) const override
                        {
                            if(fabs(z[0] - x[0]) < 0.25)
                            {
                                return 2.0;
                            }
                            return 0.0;
                        }
                    };

                    typedef BayesDistribution<X, Z> BayesXZ;

                    TEST(BayesDistributionTest, probabilityOf)
                    {
                        auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
                        auto sensor_model = std::make_shared<SensorModel>();
                        auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
                        // Probabilities are not normalized. They should be treated proportionally
                        // and not as exact probabilities.
                        EXPECT_EQ(0.0, bd->probabilityOf(0.1));
                        EXPECT_EQ(1.0, bd->probabilityOf(0.3));
                        EXPECT_EQ(1.0, bd->probabilityOf(0.5));
                        EXPECT_EQ(1.0, bd->probabilityOf(0.7));
                        EXPECT_EQ(0.0, bd->probabilityOf(0.9));
                    }

                    TEST(BayesDistributionTest, sample)
                    {
                        std::default_random_engine generator(0);
                        auto belief = std::make_shared< RangedUniform<X> >(X({0}), X({2}));
                        auto sensor_model = std::make_shared<SensorModel>();
                        auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
                        // Sample is not implemented in Bayes Distribution
                        ASSERT_THROW(bd->sample(&generator), std::runtime_error);
                    }

                    TEST(BayesDistributionTest, chainObservation)
                    {
                        auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
                        auto sensor_model = std::make_shared<SensorModel>();
                        auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
                        bd = bd->chainObservation(0.9);
                        // Probabilities are not normalized. They should be treated proportionally
                        // and not as exact probabilities.
                        EXPECT_EQ(0.0, bd->probabilityOf(0.1));
                        EXPECT_EQ(0.0, bd->probabilityOf(0.3));
                        EXPECT_EQ(0.0, bd->probabilityOf(0.5));
                        EXPECT_EQ(2.0, bd->probabilityOf(0.7));
                        EXPECT_EQ(0.0, bd->probabilityOf(0.9));
                    }

                    TEST(BayesDistributionTest, estimateNormalizer)
                    {
                        auto belief = std::make_shared< RangedUniform<X> >(X({0}), X({2}));
                        auto sensor_model = std::make_shared<SensorModel>();
                        auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
                        std::vector<X> samples;
                        double dx = 0.02;
                        samples.reserve((long)(4 / dx));
                        for(double x = -1.0; x < 3.0; x += dx)
                        {
                            samples.push_back(x);
                        }
                        EXPECT_EQ(0.5, bd->estimateNormalizer(samples, 0.02));
                        EXPECT_EQ(0.5, bd->normalizer());
                        EXPECT_EQ(0.0, bd->probabilityOf(0.1));
                        EXPECT_EQ(2.0, bd->probabilityOf(0.3));
                        EXPECT_EQ(2.0, bd->probabilityOf(0.5));
                        EXPECT_EQ(2.0, bd->probabilityOf(0.7));
                        EXPECT_EQ(0.0, bd->probabilityOf(0.9));
                    }

                    TEST(BayesDistributionTest, properties)
                    {
                        auto belief = std::make_shared<RangedUniform<X>>(X({0}), X({2}));
                        auto sensor_model = std::make_shared<SensorModel>();
                        auto bd = std::make_shared<BayesXZ>(belief, sensor_model, 0.5);
                        EXPECT_EQ(sensor_model.get(), &bd->sensorModel());
                        EXPECT_EQ(Z(0.5), bd->data());
                    }
                }
            }
        }
    }
}