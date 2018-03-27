#ifndef GAUSSIAN_SENSOR_MODEL_HPP
#define GAUSSIAN_SENSOR_MODEL_HPP

#include "pose/matrix.hpp"
#include "pose/sensor_model.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename Z, typename X>
            class GaussianSensorModel : public SensorModel<Z, X>
            {
            public:
                GaussianSensorModel()
                {
                }

                virtual ~GaussianSensorModel() override
                {
                }

                virtual Z getMean(const X& state) const = 0;

                virtual Matrix getError(const Z& observation) const = 0;
            };
        }
    }
}

#endif //GAUSSIAN_SENSOR_MODEL_HPP
