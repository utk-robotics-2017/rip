#include "settings.hpp"

#include <QStandardPaths>
#include <QDir>

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            std::shared_ptr<Settings> Settings::m_singleton = nullptr;

            std::shared_ptr<Settings> Settings::getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::shared_ptr<Settings>(new Settings);
                }
                return m_singleton;
            }

            void Settings::load()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                // Load all the motor configs
                QString motor_save_location = save_location + QDir::separator() + "motors";
                QDir motor_dir(motor_save_location);
                if (motor_dir.exists())
                {
                    QStringList files = motor_dir.entryList(QDir::Files);
                    for (const QString& filename : files)
                    {
                        QFile f(motor_save_location + QDir::separator() + filename);
                        if (f.open(QIODevice::ReadOnly))
                        {
                            nlohmann::json j = nlohmann::json::parse(f.readAll());
                            m_motors[j["name"]] = std::make_shared<misc::SettingsBase>(j);
                        }
                    }
                }
                else
                {
                    motor_dir.mkpath(".");
                }

            }

            void Settings::save()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                // Save motor settings
                for (auto iter : m_motors)
                {
                    QString filepath = save_location + QDir::separator() + "motors" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::addMotor(const std::string& name)
            {
                m_motors[name] = std::make_shared<misc::SettingsBase>(name);
                emit newMotor();
                return m_motors[name];
            }

            std::shared_ptr<misc::SettingsBase> Settings::motor(const std::string& name)
            {
                if (m_motors.find(name) == m_motors.end())
                {
                    return nullptr;
                }
                return m_motors[name];
            }

            void Settings::removeMotor(const std::string& name)
            {
                if (m_motors.find(name) != m_motors.end())
                {
                    m_motors.erase(name);
                }
            }

            std::vector<std::string> Settings::motorNames()
            {
                std::vector<std::string> names;
                for (auto it : m_motors)
                {
                    names.push_back(it.first);
                }
                return names;
            }

        }
    }
}
