#include <units/units.hpp>
#include <chrono>
namespace rip
{
    namespace units
    {

        std::chrono::time_point<std::chrono::system_clock> chrono_time_start = std::chrono::system_clock::now();

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
            // return std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() * ms;
            std::chrono::system_clock::duration time_diff = std::chrono::system_clock::now() - chrono_time_start;
            return std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count() * ms;
        }

    }
}
