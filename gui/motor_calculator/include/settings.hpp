#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <vector>
#include <memory>
#include <string>

#include <QObject>

#include <misc/settings_base.hpp>

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class Settings : public QObject
            {
                Q_OBJECT
            public:
                static std::shared_ptr<Settings> getInstance();

                void load();

                void save();

                std::shared_ptr<misc::SettingsBase> addMotor(const std::string& name);

                std::shared_ptr<misc::SettingsBase> motor(const std::string& name);

                void removeMotor(const std::string& name);

                std::vector< std::string > motorNames();

            signals:
                void newMotor();

            private:
                Settings() = default;

                static std::shared_ptr<Settings> m_singleton;

                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_motors;
            };
        }
    }
}

#endif // SETTINGS_HPP
