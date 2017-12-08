#ifndef INSTANT_ACTION_HPP
#define INSTANT_ACTION_HPP

namespace rip
{
    namespace scheduler
    {
        /**
         * An action which runs {@link Action#execute()} once
         *
         * @note This class cannot be implement as an object.
         * It must be inheritted and the other pure virtual
         * functions must be implemented
         */
        class InstantAction: public Action
        {
        public:
            InstantAction(std::string name);

        protected:
            bool isFinished() override;
        };
    }
}

#endif // INSTANT_ACTION_HPP