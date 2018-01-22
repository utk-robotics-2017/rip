#ifndef VERTEX_TIME_DIFF_HPP
#define VERTEX_TIME_DIFF_HPP

#include <units/units.hpp>

#include <g2o/core/base_vertex.h>

#include <eigen3/Eigen/Core>

namespace rip
{
    namespace navigation
    {
        class VertexTimeDiff : public g2o::BaseVertex<1 , units::Time>
        {
        public:
            VertexTimeDiff(bool fixed = false);

            VertexTimeDiff(const units::Time& dt, bool fixed = false);

            units::Time dt() const;

            void setDt(const units::Time& dt);

            /**
             * @note Required
             */
            virtual void setToOriginImpl() override;


            /**
             * @note Required
             */
            virtual void oplusImpl(const double* update) override;

            /**
             * @note Required
             */
            virtual bool read(std::istream &is) override;

            /**
             * @note Required
             */
            virtual bool write(std::ostream &os) const override;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // VERTEX_TIME_DIFF_HPP
