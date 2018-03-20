#include <gtest/gtest.h>
#include <localization/probability/data/vector.hpp>
#include <localization/probability/ranged_uniform.hpp>
#include <memory>

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

                    TEST(RangedUniformTest, ProbabilityOf)
                    {
                        auto x = std::make_shared<RangedUniform < X>>
                        (X({0.25}), X({0.75}));
                        EXPECT_EQ(0.0, x->probabilityOf(0.1));
                        EXPECT_EQ(2.0, x->probabilityOf(0.3));
                        EXPECT_EQ(2.0, x->probabilityOf(0.5));
                        EXPECT_EQ(2.0, x->probabilityOf(0.7));
                        EXPECT_EQ(0.0, x->probabilityOf(0.9));
                    }

                    TEST(RangedUniformTest, Sample)
                    {
                        std::default_random_engine generator(0);
                        auto x = std::make_shared<RangedUniform < X>>
                        (X({0.25}), X({0.75}));
                        for(int i = 0; i < 1000; i ++)
                        {
                            X sample = x->sample(&generator);
                            EXPECT_TRUE(sample[0] < 0.75 && sample[0] > 0.25);
                        }
                    }
                }
            }
        }
    }
}
