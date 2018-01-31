#include "wait_action.hpp"

namespace rip
{
    namespace core
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

            void WaitAction::setup()
            {
                m_start_time = std::chrono::system_clock::now();
            }

            void WaitAction::update() {}

            void WaitAction::teardown() {}

            nlohmann::json WaitAction::save() const
            {
                nlohmann::json j;
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                j["elapsed_time"] = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count() * units::ms;
                return j;
            }

            void WaitAction::restore(const nlohmann::json& state)
            {
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                m_start_time = now - std::chrono::milliseconds(static_cast<int>(state["elapsed_time"].get<units::Time>().to(units::ms)));
            }
        }
    }
}
