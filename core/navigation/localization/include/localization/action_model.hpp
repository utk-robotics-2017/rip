#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template <typename U, typename X>
            class ActionModel
            {
            public:
                ActionModel() = default;
                virtual ~ActionModel() = default;
            };
        }
    }
}

#endif // ACTION_MODEL_HPP
