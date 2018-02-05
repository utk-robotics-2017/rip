#ifndef SRC_CONTINUOUSANGLETRACKER_H_
#define SRC_CONTINUOUSANGLETRACKER_H_

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            /* Calculate delta angle, adjusting appropriately
             * if the current sample crossed the -180/180
             * point.
             * If the first received sample is negative,
             * ensure that the zero crossing count is
             * decremented.
             */
            class ContinuousAngleTracker
            {
                float last_angle;
                double last_rate;
                int zero_crossing_count;
                bool first_sample;
            public:
                ContinuousAngleTracker();
                void nextAngle( float newAngle );
                double getAngle();
                double getRate();
            };
        }
    }
}
#endif /* SRC_CONTINUOUSANGLETRACKER_H_ */
