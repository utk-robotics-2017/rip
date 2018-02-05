#ifndef SRC_OFFSETTRACKER_H_
#define SRC_OFFSETTRACKER_H_

namespace rip
{
    namespace navigation
    {
        namespace navx
        {

            class OffsetTracker
            {
                float *value_history;
                int next_value_history_index;
                int history_len;
                double value_offset;

            public:
                OffsetTracker(int history_length);
                void updateHistory(float curr_value);
                void setOffset();
                double applyOffset( double value );

            private:
                double getAverageFromHistory();
                double getOffset();
            };
        }//navx
    }//navigation
}//rip
#endif /* SRC_OFFSETTRACKER_H_ */
