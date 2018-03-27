#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename U, typename X>
            class ActionModel
            {
            public:
                ActionModel()
                {
                }

                virtual ~ActionModel()
                {
                }
            };
        }
    }
}

#endif //ACTION_MODEL_HPP
