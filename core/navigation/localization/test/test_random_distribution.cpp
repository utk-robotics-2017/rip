#include <gtest/gtest.h>
#include <localization/probability/random_distribution.hpp>
#include <localization/probability/data/vector.hpp>

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

                    class Impulse : RandomDistribution<X>
                    {
                    public:
                        double probabilityOf(const X& x) const override
                        {
                            if(x[0] == 0.0)
                                return 1.0;
                            return 0.0;
                        }

                        X sample(std::default_random_engine*) const override
                        {
                            return 0.0;
                        }
                    };

                    TEST(RandomDistributionTest, ctor1)
                    {
                        std::default_random_engine generator(0);
                        Impulse d;
                        ASSERT_EQ(1.0, d.probabilityOf(0.0));
                        ASSERT_EQ(0.0, d.probabilityOf(0.5));
                        ASSERT_EQ(X(0.0), d.sample(&generator));
                    }
                }
            }
        }
    }
}
