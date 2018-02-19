#include <spline_planner_gui/preferences.hpp>

#include <fstream>

#include <QStandardPaths>
#include <QDir>
#include <QTextStream>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            std::shared_ptr<Preferences> Preferences::m_singleton = nullptr;

            std::shared_ptr<Preferences> Preferences::getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::shared_ptr<Preferences>(new Preferences);
                }
                return m_singleton;
            }

            void Preferences::load()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QDir::separator() + "pref.json";

                QFile file(save_location);
                if(file.exists())
                {
                    file.open(QIODevice::ReadOnly);

                    nlohmann::json j;
                    j = nlohmann::json::parse(file.readAll().toStdString());
                    if(j.find("distance") != j.end())
                    {
                        setDistanceUnit(j["distance"].get<Distance>());
                    }

                    if(j.find("time") != j.end())
                    {
                        setTimeUnit(j["time"].get<Time>());
                    }

                    if(j.find("angle") != j.end())
                    {
                        setAngleUnit(j["angle"].get<Angle>());
                    }
                }
            }

            void Preferences::save()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QDir::separator() + "pref.json";

                QFile file(save_location);
                file.open(QIODevice::WriteOnly);

                nlohmann::json j;
                j["distance"] = m_distance_unit;
                j["time"] = m_time_unit;
                j["angle"] = m_angle_unit;

                file.write(j.dump().c_str());

            }

            units::Distance Preferences::getDistanceUnit() const
            {
                return m_distance_unit;
            }

            units::Time Preferences::getTimeUnit() const
            {
                return m_time_unit;
            }

            units::Angle Preferences::getAngleUnit() const
            {
                return m_angle_unit;
            }

            std::string Preferences::getDistanceUnitText() const
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

            std::string Preferences::getTimeUnitText() const
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

            std::string Preferences::getAngleUnitText() const
            {
                if(m_angle_unit == units::degree)
                {
                    return "degree";
                }
                else if(m_angle_unit == units::radian)
                {
                    return "radian";
                }
            }

            void Preferences::setDistanceUnit(const QString& unit)
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

            void Preferences::setDistanceUnit(const Distance& unit)
            {
                Distance temp = m_distance_unit;
                m_distance_unit = unit;
                emit distanceUnitChanged(m_distance_unit, temp);
            }

            void Preferences::setTimeUnit(const QString& unit)
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

            void Preferences::setTimeUnit(const Time& unit)
            {
                Time temp = m_time_unit;
                m_time_unit = unit;
                emit timeUnitChanged(m_time_unit, temp);
            }

            void Preferences::setAngleUnit(const QString &unit)
            {
                if(unit == "degree")
                {
                    setAngleUnit(units::degrees);
                }
                else if(unit == "radian")
                {
                    setAngleUnit(units::radian);
                }
            }

            void Preferences::setAngleUnit(const Angle &unit)
            {
                Angle temp = m_angle_unit;
                m_angle_unit = unit;
                emit angleUnitChanged(m_angle_unit, temp);
            }

            Preferences::Preferences()
                : m_distance_unit(units::in)
                , m_time_unit(units::s)
                , m_angle_unit(units::degree)
            {

            }

        }
    }
}



