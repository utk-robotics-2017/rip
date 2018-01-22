#include <teb_planner/vertex_time_diff.hpp>

namespace rip
{
    namespace navigation
    {
        VertexTimeDiff::VertexTimeDiff(bool fixed)
        {
            setFixed(fixed);
        }

        VertexTimeDiff::VertexTimeDiff(const units::Time& dt, bool fixed)
        {
            _estimate = dt;
            setFixed(fixed);
        }

        units::Time VertexTimeDiff::dt() const
        {
            return _estimate;
        }

        void VertexTimeDiff::setDt(const units::Time& dt)
        {
            _estimate = dt;
        }

        void VertexTimeDiff::setToOriginImpl()
        {
            _estimate = 0;
        }

        void VertexTimeDiff::oplusImpl(const double* update)
        {
            if(update != nullptr)
            {
                _estimate += units::Time(update[0]);
            }
        }

        bool VertexTimeDiff::read(std::istream& is)
        {
            double t;
            is >> t;
            _estimate = t;
            return true;
        }

        bool VertexTimeDiff::write(std::ostream& os) const
        {
            os << _estimate;
            return os.good();
        }
    }
}
