#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <QPainter>

#include <units/units.hpp>
#include <geometry/polygon.hpp>
#include <path_follower/rigid_transform_2d.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                /**
                 * Container for the robot's information needed for path planning
                 */
                class Robot
                {
                public:
                    Robot() = default;

                    void draw(QPainter& painter, const RigidTransform2d& pose) const;

                    std::string name() const;
                    void setName(const std::string& name);

                    geometry::Polygon shape() const;
                    void setShape(const geometry::Polygon& shape);
                    void setPoint(int index, const geometry::Point& point);

                    units::Velocity maxSpeed() const;
                    void setMaxSpeed(const units::Velocity& max_speed);

                    units::Acceleration maxAcceleration() const;
                    void setMaxAcceleration(const units::Acceleration& max_acceleration);

                    units::Distance wheelbase() const;
                    void setWheelbase(const units::Distance& wheelbase);

                    void save(const QString& filepath) const;

                    static std::shared_ptr<Robot> load(const QString& filepath);
                private:

                    std::string m_name;
                    geometry::Polygon m_shape;
                    units::Velocity m_max_speed;
                    units::Acceleration m_max_acceleration;
                    units::Distance m_wheelbase;
                };
            }
        }
    }
}

#endif // ROBOT_HPP
