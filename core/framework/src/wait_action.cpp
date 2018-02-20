#include "framework/wait_action.hpp"

namespace rip
{
    namespace framework
    {
        WaitAction::WaitAction(const units::Time& wait_time)
            : m_wait_time(wait_time)
        {}

        bool WaitAction::isFinished()
        {
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() >= m_wait_time.to(units::ms);
        }

        void WaitAction::setup(nlohmann::json& state)
        {
            m_start_time = std::chrono::system_clock::now();
        }

        void WaitAction::update(nlohmann::json& state) {}

        void WaitAction::teardown(nlohmann::json& state) {}
    }
}
