#ifndef LOOP_ACTION_HPP
#define LOOP_ACTION_HPP

#include "framework/action.hpp"
#include "framework/loop_condition.hpp"

namespace rip
{
    namespace framework
    {
        /**
         * Action for looping an action until a condition is met
         */
        class LoopAction : public Action
        {
        public:
            LoopAction(const std::string& name, std::shared_ptr<Action> action, std::shared_ptr<Action> reset_action, std::shared_ptr<LoopCondition> condition);

            virtual void setup(nlohmann::json& state) override;
            virtual void update(nlohmann::json& state) override;
            virtual void teardown(nlohmann::json& state) override;
            virtual bool isFinished() override;

        private:
            std::shared_ptr<Action> m_action;
            std::shared_ptr<Action> m_reset_action;
            std::shared_ptr<LoopCondition> m_condition;

            bool m_reset;
            bool m_previous_reset;

            bool m_finished;
        };
    }
}

#endif //LOOP_ACTION_HPP
