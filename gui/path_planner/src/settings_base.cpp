#include "settings_base.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            void to_json(nlohmann::json& j, const SettingsBase& s)
            {
                j["name"] = s.m_name;
                j["settings"] = s.m_settings;
            }

            void from_json(const nlohmann::json& j, SettingsBase& s)
            {
                s.m_name = j["name"];
                s.m_settings = j["settings"];
            }
        }
    }
}
