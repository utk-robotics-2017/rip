#ifndef GAUSSIAN_SENSOR_MODEL_HPP
#define GAUSSIAN_SENSOR_MODEL_HPP

#include "probability/data/matrix.hpp"
#include "sensor_model.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template<typename Z, typename X>
            class GaussianSensorModel : public SensorModel<Z, X>
            {
                using probability::Matrix;
            public:
                GaussianSensorModel() {}
                virtual ~GaussianSensorModel() override {}

                virtual Z getMean(const X& state) const = 0;
                virtual Matrix getError(const Z& observation) const = 0;
            };
        }
    }
}
#endif // GAUSSIAN_SENSOR_MODEL_HPP