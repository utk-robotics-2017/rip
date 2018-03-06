#ifndef FILTER_HPP
#define FILTER_HPP

#include <memory>

#include "pid_input.hpp"

namespace rip
{
    namespace pid
    {
        /**
         * An interface for the filters
         */
        class Filter : public PidInput
        {
        public:
            Filter(PidInput* input);

            /**
             * Reset the filter state
             */
            virtual void reset() = 0;

        protected:
            PidInput* m_input;
        };
    }
}

#endif // FILTER_HPP
