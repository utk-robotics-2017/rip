#ifndef SRC_CONTINUOUSANGLETRACKER_H_
#define SRC_CONTINUOUSANGLETRACKER_H_

namespace rip
{
    namespace navigation
    {
        namespace navx
        {


            class ContinuousAngleTracker {
                float last_angle;
                double last_rate;
                int zero_crossing_count;
                bool first_sample;
            public:
                ContinuousAngleTracker();
                void NextAngle( float newAngle );
                double GetAngle();
                double GetRate();
            };
        }
    }
}
#endif /* SRC_CONTINUOUSANGLETRACKER_H_ */
