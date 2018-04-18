#ifndef WORLD_HPP
#define WORLD_HPP

#include <vector>

#include <geometry/polygon.hpp>

#include "obstacle.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class World
                {
                public:
                    World() = default;

                    void draw(QPainter& painter) const;

                    std::string name() const;
                    void setName(const std::string& name);

                    geometry::Polygon boundary() const;
                    void setBoundary(const geometry::Polygon& polygon);
                    void setBoundaryPoint(int index, const geometry::Point& point);

                    std::vector< std::shared_ptr<Obstacle> > obstacles() const;
                    std::shared_ptr<Obstacle> obstacle(int index) const;
                    size_t numObstacles() const;
                    void addObstacle(std::shared_ptr<Obstacle> obstacle);

                    void save(const QString& filepath) const;

                    static std::shared_ptr<World> load(const QString& filepath);

                private:
                    nlohmann::json obstacleJson() const;

                    std::string m_name;
                    geometry::Polygon m_boundary;
                    std::vector< std::shared_ptr<Obstacle> > m_obstacles;
                };
            }
        }
    }
}

#endif // WORLD_HPP
