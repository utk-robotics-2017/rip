#ifndef GAUSSIAN_ACTION_MODEL_HPP
#define GAUSSIAN_ACTION_MODEL_HPP

#include "probability/data/matrix.hpp"
#include "action_model.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template<typename U, typename X>
            class GaussianActionModel : public ActionModel<U, X>
            {
                using probability::Matrix;
            public:
                GaussianActionModel() {
                }
                virtual ~GaussianActionModel() {}

                // Expected next state.
                virtual X getMean(const U& action, const X& state) const = 0;
                // Process noise.
                virtual Matrix getError(const U& action, const X& mean) const = 0;
            };
        }
    }
}

#endif // GAUSSIAN_ACTION_MODEL_HPP