#ifndef LOOP_CONDITION_HPP
#define LOOP_CONDITION_HPP

namespace rip
{
    namespace framework
    {
        class LoopCondition
        {
        public:
            virtual bool loop() = 0;
        };
    }
}

#endif //LOOP_CONDITION_HPP
