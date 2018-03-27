#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename Z, typename X>
            class SensorModel
            {
            public:
                SensorModel()
                {
                }

                virtual ~SensorModel()
                {
                }

                virtual double conditionalProbabilityOf(const Z& observation, const X& state) const = 0;
            };
        }
    }
}

#endif //SENSOR_MODEL_HPP
