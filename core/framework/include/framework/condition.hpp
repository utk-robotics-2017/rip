#ifndef CONDITION_HPP
#define CONDITION_HPP

namespace rip
{
    namespace framework
    {
        class Condition
        {
        public:
            virtual bool isTrue() = 0;
        };
    }
}

#endif //CONDITION_HPP
