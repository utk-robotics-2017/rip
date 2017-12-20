#ifndef SETTINGS_BASE_HPP
#define SETTINGS_BASE_HPP

#include <string>

#include <json.hpp>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class SettingsBase
            {
            public:
                SettingsBase(const std::string& name)
                    : m_name(name)
                {
                }

                std::string name() const
                {
                    return m_name;
                }

                template<typename Type>
                void set(const std::string& key, const Type& value)
                {
                    m_settings[key] = value;
                }

                template<typename Type>
                Type get(const std::string& key) const
                {
                    return m_settings[key].get<Type>();
                }

            friend void to_json(nlohmann::json& j, const SettingsBase& s);
            friend void from_json(const nlohmann::json& j, SettingsBase& s);

            private:
                std::string m_name;
                nlohmann::json m_settings;
            };
        }
    }
}

#endif // SETTINGS_BASE_HPP
