#ifndef UNITS_HPP
#define UNITS_HPP

#include <ostream>
#include <sstream>
#include <string>

#include <json.hpp>

namespace rip
{

    namespace units
    {
        using NT = float;   // Number Type

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        class Units
        {
        public:
            Units(NT value_ = NT(0)) : value(value_)
            {}

            // This turns the class into a function object that allows
            // the user to easily get at the value.
            NT operator()() const
            {
                return value;
            }

            // Helper function to get a text representation of the
            // object's dimensions.  It is static because the
            // representation is known at compile time.
            static std::string dim(void)
            {
                std::stringstream s;
                s << "<" << U1 << "," << U2 << "," << U3 << "," << U4 << "," << U5 << "," << U6 << ">";
                return s.str();
            }

            // Helper function for unit conversions.
            NT to(const Units& u) const
            {
                return value / u.value;
            }

            Units& operator=(const Units& rhs)
            {
                value = rhs.value;
                return *this;
            }

            // Arithmetic operators
            Units& operator+=(const Units& rhs)
            {
                value += rhs.value;
                return *this;
            }

            Units& operator-=(const Units& rhs)
            {
                value -= rhs.value;
                return *this;
            }

            Units& operator*=(const NT& rhs)
            {
                value *= rhs;
                return *this;
            }

            Units& operator/=(const NT& rhs)
            {
                value /= rhs;
                return *this;
            }

        private:
            NT value;
        };


        // Addition
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6>
        operator+(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return Units<U1, U2, U3, U4, U5, U6>(lhs() + rhs());
        }


        // Subtraction
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6>
        operator-(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return Units<U1, U2, U3, U4, U5, U6>(lhs() - rhs());
        }

        //Negation
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6>
        operator-(const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return Units<U1, U2, U3, U4, U5, U6>(- rhs());
        }


        // Multiplication
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6> operator*(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return Units<U1, U2, U3, U4, U5, U6>(lhs * rhs());
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6> operator*(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return rhs * lhs;
        }

        template<int U1a, int U2a, int U3a, int U4a, int U5a, int U6a, int U1b, int U2b, int U3b, int U4b, int U5b, int U6b>
        const Units < U1a + U1b, U2a + U2b, U3a + U3b, U4a + U4b, U5a + U5b, U6a + U6b >
        operator*(const Units<U1a, U2a, U3a, U4a, U5a, U6a>& lhs, const Units<U1b, U2b, U3b, U4b, U5b, U6b>& rhs)
        {
            return Units < U1a + U1b, U2a + U2b, U3a + U3b, U4a + U4b, U5a + U5b, U6a + U6b > (lhs() * rhs());
        }


        // Division
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units<U1, U2, U3, U4, U5, U6> operator/(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return Units<U1, U2, U3, U4, U5, U6>(lhs() / rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        const Units < - U1, - U2, - U3, - U4, - U5, - U6 > operator/(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return Units < - U1, - U2, - U3, - U4, - U5, - U6 > (lhs / rhs());
        }

        template<int U1a, int U2a, int U3a, int U4a, int U5a, int U6a, int U1b, int U2b, int U3b, int U4b, int U5b, int U6b>
        const Units < U1a - U1b, U2a - U2b, U3a - U3b, U4a - U4b, U5a - U5b, U6a - U6b >
        operator/(const Units<U1a, U2a, U3a, U4a, U5a, U6a>& lhs, const Units<U1b, U2b, U3b, U4b, U5b, U6b>& rhs)
        {
            return Units < U1a - U1b, U2a - U2b, U3a - U3b, U4a - U4b, U5a - U5b, U6a - U6b > (lhs() / rhs());
        }


        // Comparisons
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator==(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return fabs(lhs() - rhs()) < std::numeric_limits<NT>::epsilon();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator==(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return fabs(lhs() - rhs) < std::numeric_limits<NT>::epsilon();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator==(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return fabs(lhs - rhs()) < std::numeric_limits<NT>::epsilon();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator!=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return not (lhs() == rhs());
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator!=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return not (lhs() == rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator!=(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return not (lhs == rhs());
        }


        // Ordering
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return lhs() <= rhs();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return (lhs() <= rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<=(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return (lhs <= rhs());
        }


        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return lhs() >= rhs();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>=(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return (lhs() >= rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>=(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return (lhs >= rhs());
        }


        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return lhs() < rhs();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return (lhs() < rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator<(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return (lhs < rhs());
        }


        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return lhs() > rhs();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>(const Units<U1, U2, U3, U4, U5, U6>& lhs, const NT& rhs)
        {
            return (lhs() > rhs);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        bool operator>(const NT& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return (lhs > rhs());
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        std::ostream& operator<<(std::ostream& s, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return s << rhs();
        }

        // JSON Conversions
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        void from_json(const nlohmann::json& j, Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            rhs = j.get<NT>();
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        void to_json(nlohmann::json& j, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            j = rhs();
        }

        // math operations
        template<int U1, int U2, int U3, int U4, int U5, int U6>
        Units<U1, U2, U3, U4, U5, U6> max(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return std::max(lhs(), rhs());
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        Units<U1, U2, U3, U4, U5, U6> min(const Units<U1, U2, U3, U4, U5, U6>& lhs, const Units<U1, U2, U3, U4, U5, U6>& rhs)
        {
            return std::min(lhs(), rhs());
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        Units < U1 / 2, U2 / 2, U3 / 2, U4 / 2, U5 / 2, U6 / 2 > sqrt(const Units<U1, U2, U3, U4, U5, U6>& lhs)
        {
            return std::sqrt(lhs());
        }

        template<int exponent, int U1, int U2, int U3, int U4, int U5, int U6>
        auto pow(const Units<U1, U2, U3, U4, U5, U6>& base) -> Units<U1* exponent, U2* exponent, U3* exponent, U4* exponent, U5* exponent, U6* exponent>
        {
            return std::pow(base(), exponent);
        }

        template<int U1, int U2, int U3, int U4, int U5, int U6>
        Units<U1, U2, U3, U4, U5, U6> abs(const Units<U1, U2, U3, U4, U5, U6>& lhs)
        {
            return std::fabs(lhs());
        }

        // operator>> is not provided because the unit type can not be
        // created at runtime in any reasonable fashion.  This means there is
        // no easy way to serialize unit objects.
        //
        // If you need to read in an object from a stream, read it into an NT
        // variable and put it into an appropriate-type variable.  Example:
        //
        //      NT x;
        //      cin >> x;
        //      Length y = x*m;
        //
        // where the base unit m has already been defined.  This requires you
        // to i) know the unit type at compile time and ii) assume its value
        // is in terms of the base type.

        // Typedefs for fundamental units
        typedef Units<1, 0, 0, 0, 0, 0> Length;
        typedef Length Distance;
        typedef Units<0, 1, 0, 0, 0, 0> Time;
        typedef Units<0, 0, 1, 0, 0, 0> Mass;
        typedef Units<0, 0, 0, 1, 0, 0> Current;
        typedef Units<0, 0, 0, 0, 1, 0> Temperature;
        typedef Units<0, 0, 0, 0, 0, 1> Angle;

        // Typedefs for derived units
        typedef Units<2, 0, 0, 0, 0, 0> Area;
        typedef Units < 1, - 1, 0, 0, 0, 0 > Velocity;
        typedef Units < 1, - 2, 0, 0, 0, 0 > Acceleration;
        typedef Units < 1, - 3, 0, 0, 0, 0 > Jerk;
        typedef Units < 0, - 1, 0, 0, 0, 1 > AngularVelocity;
        typedef Units < 0, - 2, 0, 0, 0, 1 > AngularAcceleration;
        typedef Units < 2, - 3, 1, - 1, 0, 0 > Voltage;
        typedef Units<0, 0, 0, 0, 0, 0> Unitless;

        // Unit constants
        const double pi = 3.14159265358;
        const NT tera = 1e12;
        const NT giga = 1e9;
        const NT mega = 1e6;
        const NT kilo = 1e3;
        const NT deci = 1e-1;
        const NT centi = 1e-2;
        const NT milli = 1e-3;
        const NT micro = 1e-6;
        const NT nano = 1e-9;
        const NT pico = 1e-12;
        const NT femto = 1e-15;
        const NT atto = 1e-18;
        const Length micron = 1;
        const Length m = mega * micron;
        const Length km = kilo * m;
        const Length cm = centi * m;
        const Length mm = milli * m;
        const Length in = 2.54 * cm;
        const Length inch = in;
        const Length inches = in;
        const Length ft = 12 * in;
        const Length foot = ft;
        const Length feet = ft;
        const Area m2 = 1 * m * m;
        const Area mm2 = 1 * mm * mm;
        const Area cm2 = 1 * cm * cm;
        const Area in2 = 1 * in * in;
        const Area ft2 = 1 * ft * ft;
        const Angle radian = 1;
        const Angle rad = 1;
        const Angle deg = 1.74532925e-2 * rad;
        const Angle degree = deg;
        const Angle degrees = deg;
        const Angle rev = 2 * pi * rad;
        const Mass kg = 1;
        const Mass g = milli * kg;
        const Mass mg = milli * g;
        const Time s = 1;
        const Time ms = milli * s;
        const Time hr = 3600 * s;
        const Time hour = hr;
        const Time minute = 60 * s;
        const Current A = 1;
        const Voltage V = kg * m * m / (A* s* s* s);
        const Temperature K = 1;
        const Temperature degC = 1 * K;
        const Temperature degF = 5 / 9 * K;
        const Acceleration AccelerationOfGravity = 9.80665 * m / (s* s);
        const Unitless none = 1;

        double cos(const Angle& lhs);
        double sin(const Angle& lhs);
    }
}
#endif // UNITS_HPP
