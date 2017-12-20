#ifndef PREFERENCES_HPP
#define PREFERENCES_HPP

#include <memory>

#include <QObject>

#include <units.hpp>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class PreferencesManager : public QObject
            {
                using Distance = units::Distance;
                using Time = units::Time;

                Q_OBJECT
            public:
                static std::shared_ptr<PreferencesManager> getInstance();

                void load();

                void save();

                Distance getDistanceUnit() const;

                Time getTimeUnit() const;

                std::string getDistanceUnitText() const;

                std::string getTimeUnitText() const;

            signals:
                /**
                 * Signal emitted when the distance unit is changed
                 */
                void distanceUnitChanged(const Distance& new_value, const Distance& old_value);

                /**
                 * Signal emitted when the time unit is changed
                 */
                void timeUnitChanged(const Time& new_value, const Time& old_value);

            public slots:
                /**
                 * Sets the unit used for Distance/Length
                 */
                void setDistanceUnit(const QString& unit);

                void setDistanceUnit(const Distance& unit);

                /**
                 * Sets the unit used for Time
                 */
                void setTimeUnit(const QString& unit);

                /**
                 * Sets the unit used for Time
                 */
                void setTimeUnit(const Time& unit);

            private:
                PreferencesManager() = default;

                static std::shared_ptr<PreferencesManager> m_singleton;

                Distance m_distance_unit;
                // Angle m_angle_unit;
                Time m_time_unit;
            };
        }
    }
}

#endif // PREFERENCES_HPP
