#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <vector>
#include <ostream>

#include "matrix.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class Vector
            {
            public:
                Vector();

                explicit Vector(std::vector<double> a);

                explicit Vector(const Vector& v);

                Vector(Vector&& v);

                explicit Vector(std::size_t order);

                virtual ~Vector();

                std::size_t order() const;

                double& at(int i);

                double at(int i) const;

                double& operator[](std::size_t i);

                double operator[](std::size_t i) const;

                Matrix cross(const Vector& rhs) const;

                Vector& operator=(const Vector& rhs);

                virtual void set(Vector a);

                void copyAssign(const Vector& v);

                void moveAssign(Vector&& v);

                Vector aliasVector() const;

                double dot(const Vector& v2) const;

                Vector operator+(const Vector& rhs) const;

                Vector operator-(const Vector& rhs) const;

                bool operator<(const Vector& rhs) const;

                bool operator>(const Vector& rhs) const;

                bool operator==(const Vector& rhs) const;

            private:
                std::vector<double> m_a;
            };

            std::ostream& operator<<(std::ostream& os, const Vector& v);


#define DEFINE_VECTOR1(X) \
class X : public rip::navigation::pose::Vector { \
public: \
  X() : Vector(std::vector<double>({ 0.0 })) {} \
  X(double x) : Vector(std::vector<double>({ x })) {} \
  X(const X& v) : Vector(v) {} \
  X(const X&& v) : Vector(v) {} \
  X(rip::navigation::pose::Vector&& v) : Vector(v) {} \
  X& operator=(const X& that) { Vector::operator=(that); return *this; } \
}
        }
    }
}

#endif //VECTOR_HPP
