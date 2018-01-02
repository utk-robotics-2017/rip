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
            class Preferences : public QObject
            {
                using Distance = units::Distance;
                using Time = units::Time;
                using Angle = units::Angle;

                Q_OBJECT
            public:
                static std::shared_ptr<Preferences> getInstance();

                /**
                 * Loads the previously saves preferences from file
                 */
                void load();

                /**
                 * Saves the current preferences to file
                 */
                void save();

                /**
                 * Returns the preferred unit for distance
                 */
                Distance getDistanceUnit() const;

                /**
                 * Returns the preferred unit for time
                 */
                Time getTimeUnit() const;

                /**
                 * Returns the preferred unit for angle
                 */
                Angle getAngleUnit() const;

                /**
                 * Returns a string representation of the preferred unit for distance
                 */
                std::string getDistanceUnitText() const;

                /**
                 * Returns a string representation of the preferred unit for time
                 */
                std::string getTimeUnitText() const;

                /**
                 * Returns a string representation of the preferred unit for angle
                 */
                std::string getAngleUnitText() const;

            signals:
                /**
                 * Signal emitted when the distance unit is changed
                 */
                void distanceUnitChanged(const Distance& new_value, const Distance& old_value);

                /**
                 * Signal emitted when the time unit is changed
                 */
                void timeUnitChanged(const Time& new_value, const Time& old_value);

                /**
                 * Signal emitted when the angle unit is changed
                 */
                void angleUnitChanged(const Angle& new_value, const Angle& old_value);

            public slots:
                /**
                 * Sets the unit used for Distance/Length
                 */
                void setDistanceUnit(const QString& unit);

                /**
                 * Sets the unit used for Distance/Length
                 */
                void setDistanceUnit(const Distance& unit);

                /**
                 * Sets the unit used for Time
                 */
                void setTimeUnit(const QString& unit);

                /**
                 * Sets the unit used for Time
                 */
                void setTimeUnit(const Time& unit);

                /**
                 * Sets the unit used for Angle
                 */
                void setAngleUnit(const QString& unit);

                /**
                 * Sets the unit used for Angle
                 */
                void setAngleUnit(const Angle& unit);

            private:
                /**
                 * Constructor
                 */
                Preferences();

                static std::shared_ptr<Preferences> m_singleton;

                Distance m_distance_unit;
                Angle m_angle_unit;
                Time m_time_unit;
            };
        }
    }
}

#endif // PREFERENCES_HPP
