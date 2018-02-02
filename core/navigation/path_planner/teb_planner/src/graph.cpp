#include <teb_planner/graph.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            Vertex::Vertex(const Eigen::Vector2d& data)
                : m_pos(data)
            {}

            void Vertex::addEdge(std::shared_ptr<Edge> edge)
            {
                assert((*(edge->start())) == *this );
                m_adjacent_edges.push_back(edge);
            }

            std::vector<std::shared_ptr<Edge> > Vertex::adjacentEdges() const
            {
                return m_adjacent_edges;
            }

            std::vector<std::shared_ptr<Vertex> > Vertex::adjacentVertices() const
            {
                std::vector< std::shared_ptr<Vertex> > ret;
                for (std::shared_ptr<Edge> edge : m_adjacent_edges)
                {
                    ret.push_back(edge->end());
                }

                return ret;
            }

            bool Vertex::operator ==(const Vertex& rhs) const
            {
                return m_pos == rhs.m_pos;
            }

            Eigen::Vector2d Vertex::pos() const
            {
                return m_pos;
            }

            Edge::Edge(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end, float weight)
                : m_start(start)
                , m_end(end)
                , m_weight(weight)
            {}

            std::shared_ptr<Vertex> Edge::start() const
            {
                return m_start;
            }

            std::shared_ptr<Vertex> Edge::end() const
            {
                return m_end;
            }

            void Edge::setWeight(float weight)
            {
                m_weight = weight;
            }

            float Edge::weight() const
            {
                return m_weight;
            }

            Graph::Graph() {}

            std::shared_ptr<Vertex> Graph::addVertex(const Eigen::Vector2d& data)
            {
                m_vertices.push_back(std::make_shared<Vertex>(data));
                return m_vertices.back();
            }

            std::vector<std::shared_ptr<Vertex> > Graph::vertices() const
            {
                return m_vertices;
            }

            std::shared_ptr<Edge> Graph::addEdge(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end, float weight)
            {
                std::shared_ptr<Edge> edge = std::make_shared<Edge>(start, end, weight);

                start->addEdge(edge);
                m_edges.push_back(edge);

                return edge;
            }

            void Graph::clear()
            {
                m_vertices.clear();
                m_edges.clear();
            }


        }
    }
}
