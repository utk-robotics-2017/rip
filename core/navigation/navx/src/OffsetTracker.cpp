

#include <OffsetTracker.h>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {

            OffsetTracker::OffsetTracker(int history_length)
            {
                history_len = history_length;
                value_history = new float[history_len];
                for(int i = 0; i < history_len; i++)
                {
                    value_history[i] = 0.0f;
                }
                next_value_history_index = 0;
                value_offset = 0;
            }

            void OffsetTracker::updateHistory(float curr_value)
            {
                if(next_value_history_index >= history_len)
                {
                    next_value_history_index = 0;
                }
                value_history[next_value_history_index] = curr_value;
                next_value_history_index++;
            }

            double OffsetTracker::getAverageFromHistory()
            {
                double value_history_sum = 0.0;
                for(int i = 0; i < history_len; i++)
                {
                    value_history_sum += value_history[i];
                }
                double value_history_avg = value_history_sum / history_len;
                return value_history_avg;
            }

            void OffsetTracker::setOffset()
            {
                value_offset = getAverageFromHistory();
            }

            double OffsetTracker::getOffset()
            {
                return value_offset;
            }

            double OffsetTracker::applyOffset(double value) 
            {
                float offseted_value =(float)(value - value_offset);
                if(offseted_value < -180)
                {
                    offseted_value += 360;
                }
                if(offseted_value > 180)
                {
                    offseted_value -= 360;
                }
                return offseted_value;
            }
        }
    }
}
