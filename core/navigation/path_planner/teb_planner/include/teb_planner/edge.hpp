#ifndef EDGE_HPP
#define EDGE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include "teb_config.hpp"

#include <cmath>

namespace rip
{
    namespace navigation
    {
        /**
         * Base edge connecting a single vertex in the TEB optimization problem
         */
        template <int D, typename E, typename VertexXi>
        class BaseUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
        {
        public:
            using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

            /**
             * Constructor
             */
            BaseUnaryEdge()
            {
                _vertices[0] = nullptr;
            }

            /**
             * Destructor
             *
             * @note We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
             * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
             */
            virtual ~BaseUnaryEdge()
            {
                if (_vertices[0])
                {
                    _vertices[0]->edges().erase(this);
                }
            }

            /**
             * Compute and return error / cost value
             */
            ErrorVector& getError()
            {
                computeError();
                return _error;
            }

            void setConfig(std::shared_ptr<TebConfig> config)
            {
                m_config = config;
            }

            /**
             * @brief Read values from input stream
             */
            virtual bool read(std::istream& is) override
            {
              // TODO generic read
              return true;
            }

            /**
             * @brief Write values to an output stream
             */
            virtual bool write(std::ostream& os) const override
            {
              // TODO generic write
              return os.good();
            }


        protected:
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
            using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

            std::shared_ptr<TebConfig> m_config;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Base edge connecting two vertices in the TEB optimization problem
         */
        template<int D, typename E, typename VertexXi, typename VertexXj>
        class BaseBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
        {
        public:

            using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;
            /**
             * Constructor
             */
            BaseBinaryEdge()
            {
                _vertices[0] = _vertices[1] = nullptr;
            }

            /**
             * Destructor
             *
             * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
             * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
             */
            virtual~BaseBinaryEdge()
            {
                if (_vertices[0])
                {
                    _vertices[0]->edges().erase(this);
                }

                if (_vertices[1])
                {
                    _vertices[1]->edges().erase(this);
                }
            }

            /**
             * Compute and return error / cost value
             */
            ErrorVector& getError()
            {
                computeError();
                return _error;
            }

            void setConfig(std::shared_ptr<TebConfig> config)
            {
                m_config = config;
            }

            /**
             * @brief Read values from input stream
             */
            virtual bool read(std::istream& is) override
            {
              // TODO generic read
              return true;
            }

            /**
             * @brief Write values to an output stream
             */
            virtual bool write(std::ostream& os) const override
            {
              // TODO generic write
              return os.good();
            }


        protected:
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
            using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

            std::shared_ptr<TebConfig> m_config;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        template <int D, typename E>
        class BaseMultiEdge : public g2o::BaseMultiEdge<D, E>
        {
        public:
            using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
            using g2o::BaseMultiEdge<D, E>::computeError;

            BaseMultiEdge()
            {}

            virtual ~BaseMultiEdge()
            {
                for (size_t i = 0; i < _vertices.size(); i++)
                {
                    if (_vertices[i])
                    {
                        _vertices[i]->edges().erase(this);
                    }
                }
            }

            virtual void resize(size_t size) override
            {
                g2o::BaseMultiEdge<D, E>::resize(size);

                for (size_t i = 0; i < _vertices.size(); i++)
                {
                    _vertices[i] = nullptr;
                }
            }

            /**
             * Compute and return error / cost value
             */
            ErrorVector& getError()
            {
                computeError();
                return _error;
            }

            void setConfig(std::shared_ptr<TebConfig> config)
            {
                m_config = config;
            }

            /**
             * @brief Read values from input stream
             */
            virtual bool read(std::istream& is) override
            {
              // TODO generic read
              return true;
            }

            /**
             * @brief Write values to an output stream
             */
            virtual bool write(std::ostream& os) const override
            {
              // TODO generic write
              return os.good();
            }


        protected:
            using g2o::BaseMultiEdge<D, E>::_error;
            using g2o::BaseMultiEdge<D, E>::_vertices;

            std::shared_ptr<TebConfig> m_config;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_HPP
