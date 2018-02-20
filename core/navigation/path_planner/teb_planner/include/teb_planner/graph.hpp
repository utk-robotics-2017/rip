#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            class Edge;


            class Vertex
            {
            public:
                Vertex(const Eigen::Vector2d& data);

                void addEdge(std::shared_ptr<Edge> edge);

                std::vector< std::shared_ptr<Edge> > adjacentEdges() const;

                std::vector< std::shared_ptr<Vertex> > adjacentVertices() const;

                bool operator ==(const Vertex& rhs) const;

                Eigen::Vector2d pos() const;

            private:
                std::vector< std::shared_ptr<Edge> > m_adjacent_edges;
                Eigen::Vector2d m_pos;
            };

            class Edge
            {
            public:
                Edge(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end, float weight);

                std::shared_ptr<Vertex> start() const;

                std::shared_ptr<Vertex> end() const;

                void setWeight(float weight);

                float weight() const;

            private:
                std::shared_ptr<Vertex> m_start;
                std::shared_ptr<Vertex> m_end;
                float m_weight;
            };


            class Graph
            {
            public:

                Graph();

                std::shared_ptr<Vertex> addVertex(const Eigen::Vector2d& data);

                std::vector< std::shared_ptr<Vertex> > vertices() const;

                std::shared_ptr<Edge> addEdge(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end, float weight = 1);

                void clear();

            private:
                std::vector< std::shared_ptr<Vertex> > m_vertices;
                std::vector< std::shared_ptr<Edge> > m_edges;
            };
        }
    }
}

#endif // GRAPH_HPP
