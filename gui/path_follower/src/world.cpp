#include "path_follower_gui/world.hpp"
#include <QFile>

#include <fmt/format.h>

#include "path_follower_gui/exceptions.hpp"
#include "path_follower_gui/draw_utilities.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                void World::draw(QPainter& painter) const
                {
                    if(m_boundary.size() > 0)
                    {
                        painter.setBrush(Qt::white);
                        DrawUtilities::drawPolygon(painter, m_boundary);
                    }

                    painter.setBrush(Qt::black);
                    for(std::shared_ptr<Obstacle> obstacle : m_obstacles)
                    {
                        obstacle->draw(painter);
                    }
                }

                std::string World::name() const
                {
                    return m_name;
                }

                void World::setName(const std::string& name)
                {
                    m_name = name;
                }

                geometry::Polygon World::boundary() const
                {
                    return m_boundary;
                }

                void World::setBoundary(const geometry::Polygon& polygon)
                {
                    m_boundary = polygon;
                }

                void World::setBoundaryPoint(int index, const geometry::Point& point)
                {
                    assert(index >= 0 && index < m_boundary.size());
                    m_boundary[index] = point;
                }

                std::vector< std::shared_ptr<Obstacle> > World::obstacles() const
                {
                    return m_obstacles;
                }

                std::shared_ptr<Obstacle> World::obstacle(int index) const
                {
                    assert(index >= 0 && index < m_obstacles.size());
                    return m_obstacles[index];
                }

                size_t World::numObstacles() const
                {
                    return m_obstacles.size();
                }

                void World::addObstacle(std::shared_ptr<Obstacle> obstacle)
                {
                    m_obstacles.push_back(obstacle);
                }

                void World::save(const QString& filepath) const
                {
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = m_name;
                        j["boundary"] = m_boundary;
                        j["obstacles"] = obstacleJson();
                        f.write(j.dump(4).c_str());
                    }
                }

                std::shared_ptr<World> World::load(const QString& filepath)
                {
                    QFile f(filepath);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());

                        std::shared_ptr<World> world = std::make_shared<World>();

                        if(j.find("name") == j.end())
                        {
                            throw WorldConfigException("Missing name");
                        }
                        world->m_name = j["name"];

                        if(j.find("boundary") == j.end())
                        {
                            throw WorldConfigException("Missing boundary");
                        }
                        world->m_boundary = j["boundary"].get<geometry::Polygon>();

                        if(j.find("obstacles") == j.end())
                        {
                            throw WorldConfigException("Missing obstacles");
                        }
                        for(nlohmann::json o : j["obstacles"])
                        {
                            world->m_obstacles.push_back(std::make_shared<Obstacle>(o.get<geometry::Polygon>()));
                        }
                        return world;
                    }
                    else
                    {
                        throw FileNotFound(fmt::format("World config file: {} not found", filepath.toStdString()));
                    }
                }

                nlohmann::json World::obstacleJson() const
                {
                    nlohmann::json rv;
                    for(std::shared_ptr<Obstacle> obstacle : m_obstacles)
                    {
                        rv.push_back(obstacle->polygon());
                    }
                    return rv;
                }
            }
        }
    }
}
