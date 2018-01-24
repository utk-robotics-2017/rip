#ifndef VERTEX_POSE_HPP
#define VERTEX_POSE_HPP

#include <g2o/core/base_vertex.h>

#include "pose.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * This class stores and wraps a SE2 pose (position and orientation)
         * into a vertex that can be optimized via g2o
         */
        class VertexPose : public g2o::BaseVertex<3, Pose>
        {
        public:
            /**
             * Constructor
             */
            VertexPose(bool fixed = false)
            {
                setFixed(fixed);
            }

            VertexPose(const Pose& pose, bool fixed = false)
            {
                _estimate = pose;
                setFixed(fixed);
            }

            /**
             * Returns the pose for this vertex
             */
            Pose pose() const
            {
                return _estimate;
            }

            /**
             * Sets the pose for this vertex
             */
            void setPose(const Pose& pose)
            {
                _estimate = pose;
            }

            /**
             * Returns the position for this vertex
             */
            geometry::Point position() const
            {
                return _estimate.position();
            }

            /**
             * Returns the x component of the position
             * for this vertex
             */
            units::Distance x() const
            {
                return _estimate.x();
            }

            /**
             * Returns the y component of the position
             * for this vertex
             */
            units::Distance y() const
            {
                return _estimate.y();
            }

            /**
             * Returns the orientation for this vertex
             */
            units::Angle theta() const
            {
                return _estimate.theta();
            }

            /**
             * @note Required
             */
            virtual void setToOriginImpl() override
            {
                _estimate.setX(0);
                _estimate.setY(0);
                _estimate.setTheta(0);
            }

            /**
             * @note Required
             */
            virtual void oplusImpl(const double* update) override
            {
                if(update != nullptr)
                {
                    _estimate += Pose(update[0], update[1], update[2]);
                }
            }

            /**
             * @note Required
             */
            virtual bool read(std::istream &is) override
            {
                double x, y, z;
                is >> x >> y >> z;
                _estimate.setX(x);
                _estimate.setY(y);
                _estimate.setTheta(z);
                return true;
            }

            /**
             * @note Required
             */
            virtual bool write(std::ostream &os) const override
            {
                os << _estimate.x()() << " " << _estimate.y()() << " " << _estimate.theta()();
                return os.good();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // VERTEX_POSE_HPP
