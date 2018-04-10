#include <units/units.hpp>
#include <chrono>
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

        Time now()
        {
            return std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() * ms;
        }

    }
}
