/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef SETTINGS_BASE_HPP
#define SETTINGS_BASE_HPP

#include <string>

#include <json.hpp>

namespace rip
{
    namespace misc
    {

        class SettingsBase
        {
        public:
            SettingsBase(const std::string& name)
                : m_name(name)
            {
            }

            SettingsBase(const nlohmann::json& j)
            {
                m_name = j["name"];
                m_settings = j["settings"];
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

            bool contains(const std::string& key) const
            {
                if (m_settings.find(key) != m_settings.end())
                {
                    return true;
                }
                return false;
            }

            template<typename Type>
            Type get(const std::string& key) const
            {
                if (contains(key))
                {
                    return m_settings[key].get<Type>();
                }
                return Type();
            }

            friend void to_json(nlohmann::json& j, const SettingsBase& s);
            friend void from_json(const nlohmann::json& j, SettingsBase& s);

        private:
            std::string m_name;
            nlohmann::json m_settings;
        };

        void to_json(nlohmann::json& j, const SettingsBase& s);
        void from_json(const nlohmann::json& j, SettingsBase& s);
    }
}

#endif // SETTINGS_BASE_HPP
