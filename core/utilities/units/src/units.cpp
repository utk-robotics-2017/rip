#include <units/units.hpp>

namespace rip
{
    namespace units
    {
        double cos(const Angle& lhs)
        {
            return std::cos(lhs.to(rad));
        }

        double sin(const Angle& lhs)
        {
            return std::sin(lhs.to(rad));
        }
    }
}
