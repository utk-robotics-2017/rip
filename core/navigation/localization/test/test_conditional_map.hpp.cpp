#include <gtest/gtest.h>
#include <localization/probability/data/vector.hpp>
#include <localization/probability/conditional_map.hpp>
#include <localization/probability/ranged_uniform.hpp>

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
                    enum Y
                    {
                        y0,
                        y1
                    };

                    TEST(ConditionalMapTest, GetSet) {
                        ConditionalMap<X, Y> px_given_y;
                        px_given_y.set(y0, std::make_shared<RangedUniform<X>>(X(0.5), X(1.0)));
                        px_given_y.set(y1, std::make_shared<RangedUniform<X>>(X(0.0), X(2.0)));
                        EXPECT_EQ(0.0, px_given_y.conditionalProbabilityOf(0.4, y0));
                        EXPECT_EQ(2.0, px_given_y.conditionalProbabilityOf(0.8, y0));
                        EXPECT_EQ(0.0, px_given_y.conditionalProbabilityOf(1.5, y0));
                        EXPECT_EQ(0.0, px_given_y.conditionalProbabilityOf(-1.0, y1));
                        EXPECT_EQ(0.5, px_given_y.conditionalProbabilityOf(1.5, y1));
                        EXPECT_EQ(0.0, px_given_y.conditionalProbabilityOf(3.0, y1));
                    }
                }
            }
        }
    }
}