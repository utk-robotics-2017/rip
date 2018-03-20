#include <gtest/gtest.h>
#include <localization/probability/discrete/distribution_value_map.hpp>
#include <localization/probability/discrete/conditional_value_map.hpp>

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
                    enum class X
                    {
                        x0, x1
                    };

                    enum class Y
                    {
                        y0, y1
                    };

                    enum class Z
                    {
                        z0, z1
                    };

                    typedef ConditionalValueMap <X, Y> CvmXY;
                    typedef ConditionalValueMap <Y, Z> CvmYZ;

                    TEST(ConditionalValueMapTest, DomainRange)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 1.0);
                        xy.set(X::x1, Y::y0, 1.0);
                        EXPECT_EQ(std::vector<X>({X::x0, X::x1}), xy.domain());
                        EXPECT_EQ(std::vector<Y>({Y::y0}), xy.range());
                    }

                    TEST(ConditionalValueMapTest, SetGet)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 0.9);
                        xy.set(X::x1, Y::y0, 0.1);
                        xy.set(X::x0, Y::y1, 0.2);
                        xy.set(X::x1, Y::y1, 0.8);
                        EXPECT_EQ(0.9, xy.conditionalProbabilityOf(X::x0, Y::y0));
                        EXPECT_EQ(0.1, xy.conditionalProbabilityOf(X::x1, Y::y0));
                        EXPECT_EQ(0.2, xy.conditionalProbabilityOf(X::x0, Y::y1));
                        EXPECT_EQ(0.8, xy.conditionalProbabilityOf(X::x1, Y::y1));
                    }

                    TEST(ConditionalValueMapTest, LikelihoodOf)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 0.9);
                        xy.set(X::x1, Y::y0, 0.1);
                        xy.set(X::x0, Y::y1, 0.2);
                        xy.set(X::x1, Y::y1, 0.8);

                        auto lh0 = xy.likelihoodOf(X::x0);
                        EXPECT_DOUBLE_EQ(0.9, lh0->probabilityOf(Y::y0));
                        EXPECT_DOUBLE_EQ(0.2, lh0->probabilityOf(Y::y1));
                        lh0->normalize();
                        EXPECT_DOUBLE_EQ(9.0 / 11.0, lh0->probabilityOf(Y::y0));
                        EXPECT_DOUBLE_EQ(2.0 / 11.0, lh0->probabilityOf(Y::y1));

                        auto lh1 = xy.likelihoodOf(X::x1);
                        EXPECT_DOUBLE_EQ(0.1, lh1->probabilityOf(Y::y0));
                        EXPECT_DOUBLE_EQ(0.8, lh1->probabilityOf(Y::y1));
                        lh1->normalize();
                        EXPECT_DOUBLE_EQ(1.0 / 9.0, lh1->probabilityOf(Y::y0));
                        EXPECT_DOUBLE_EQ(8.0 / 9.0, lh1->probabilityOf(Y::y1));
                    }

                    TEST(ConditionalValueMapTest, BayesianInference)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 0.9);
                        xy.set(X::x1, Y::y0, 0.1);
                        xy.set(X::x0, Y::y1, 0.2);
                        xy.set(X::x1, Y::y1, 0.8);
                        DistributionValueMap <Y> belief0(Y::y0, 0.1, Y::y1, 0.9);
                        auto belief1 = xy.bayesianInference(X::x0, belief0);
                        EXPECT_DOUBLE_EQ(9.0 / 27.0, belief1->probabilityOf(Y::y0));
                        EXPECT_DOUBLE_EQ(18.0 / 27.0, belief1->probabilityOf(Y::y1));
                    }

                    TEST(ConditionalValueMapTest, Marginalize)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 0.9);
                        xy.set(X::x1, Y::y0, 0.1);
                        xy.set(X::x0, Y::y1, 0.2);
                        xy.set(X::x1, Y::y1, 0.8);

                        CvmYZ yz;
                        yz.set(Y::y0, Z::z0, 0.9);
                        yz.set(Y::y1, Z::z0, 0.1);
                        yz.set(Y::y0, Z::z1, 0.2);
                        yz.set(Y::y1, Z::z1, 0.8);

                        auto xz = xy.marginalize(yz);
                        EXPECT_DOUBLE_EQ(0.83, xz->conditionalProbabilityOf(X::x0, Z::z0));
                        EXPECT_DOUBLE_EQ(0.17, xz->conditionalProbabilityOf(X::x1, Z::z0));
                        EXPECT_DOUBLE_EQ(0.34, xz->conditionalProbabilityOf(X::x0, Z::z1));
                        EXPECT_DOUBLE_EQ(0.66, xz->conditionalProbabilityOf(X::x1, Z::z1));
                    }

                    TEST(ConditionalValueMapTest, SampleLikelihood)
                    {
                        CvmXY xy;
                        xy.set(X::x0, Y::y0, 0.9);
                        xy.set(X::x1, Y::y0, 0.1);
                        xy.set(X::x0, Y::y1, 0.2);
                        xy.set(X::x1, Y::y1, 0.8);

                        EXPECT_EQ(1.1, xy.sumLikelihood(X::x0));
                        EXPECT_EQ(0.9, xy.sumLikelihood(X::x1));

                        EXPECT_EQ(Y::y0, xy.sampleLikelihood(X::x0, 0.85));
                        EXPECT_EQ(Y::y1, xy.sampleLikelihood(X::x0, 0.95));
                        EXPECT_EQ(Y::y0, xy.sampleLikelihood(X::x1, 0.05));
                        EXPECT_EQ(Y::y1, xy.sampleLikelihood(X::x1, 0.15));
                    }
                }
            }
        }
    }
}