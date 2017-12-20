#include "preferences_manager.hpp"

#include <QStandardPaths>
#include <QDir>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            std::shared_ptr<PreferencesManager> PreferencesManager::m_singleton = nullptr;

            std::shared_ptr<PreferencesManager> PreferencesManager::getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::shared_ptr<PreferencesManager>(new PreferencesManager);
                }
                return m_singleton;
            }

            void PreferencesManager::load()
            {
                std::string save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation).toStdString() + QDir::separator().toLatin1() + "pref.json";

                FileHandle fh = fs::open(save_location);
                std::unique_ptr<std::istream> input = fh.createInputStream();

                nlohmann::json j;
                (*input) >> j;
                setDistanceUnit(j["distance"].get<Distance>());
                setTimeUnit(j["time"].get<Time>());
            }

            void PreferencesManager::save()
            {
                std::string save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation).toStdString() + QDir::separator().toLatin1() + "pref.json";

                FileHandle fh = fs::open(save_location);
                std::unique_ptr<std::ostream> output = fh.createOutputStream();

                nlohmann::json j;
                j["distance"] = m_distance_unit;
                j["time"] = m_time_unit;

                (*output) << j;
            }

            units::Distance PreferencesManager::getDistanceUnit() const
            {
                return m_distance_unit;
            }

            units::Time PreferencesManager::getTimeUnit() const
            {
                return m_time_unit;
            }

            std::string PreferencesManager::getDistanceUnitText() const
            {
                if (m_distance_unit == units::in)
                {
                    return "in";
                }
                else if (m_distance_unit == units::mm)
                {
                    return "mm";
                }
                else if (m_distance_unit == units::cm)
                {
                    return "cm";
                }

            }

            std::string PreferencesManager::getTimeUnitText() const
            {
                if (m_time_unit == units::s)
                {
                    return "s";
                }
                else if (m_time_unit == units::ms)
                {
                    return "ms";
                }
            }

            void PreferencesManager::setDistanceUnit(const QString& unit)
            {
                if (unit == "in")
                {
                    setDistanceUnit(units::in);
                }
                else if (unit == "mm")
                {
                    setDistanceUnit(units::mm);
                }
                else if (unit == "cm")
                {
                    setDistanceUnit(units::cm);
                }
            }

            void PreferencesManager::setDistanceUnit(const Distance& unit)
            {
                Distance temp = m_distance_unit;
                m_distance_unit = unit;
                emit distanceUnitChanged(m_distance_unit, temp);
            }

            void PreferencesManager::setTimeUnit(const QString& unit)
            {
                if (unit == "s")
                {
                    setTimeUnit(units::s);
                }
                else if (unit == "ms")
                {
                    setTimeUnit(units::ms);
                }
            }

            void PreferencesManager::setTimeUnit(const Time& unit)
            {
                Time temp = m_time_unit;
                m_time_unit = unit;
                emit timeUnitChanged(m_time_unit, temp);
            }

        }
    }
}



