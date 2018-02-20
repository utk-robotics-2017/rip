#ifndef PROPERTY_HPP
#define PROPERTY_HPP

namespace rip
{
    namespace property
    {
        /**
         * Read only property
         */
        template<typename T, typename Type, Type(T::*Getter)() const>
        struct PropertyR
        {
            operator Type() const
            {
                return (This()->*Getter)();
            }
        private:
            const T* This() const
            {
                return reinterpret_cast<const T*>(this);
            }
        };

        /**
         * Write only property
         */
        template<typename T, typename Type, void (T::*Setter)(Type)>
        struct PropertyW
        {
            PropertyW<T, Type, Setter> operator = (Type t)
            {
                (This()->*Setter)(t);
                return *this;
            }
        private:
            T* This()
            {
                return reinterpret_cast<T*>(this);
            }
        };

        template <typenameT, typename Type, Type(T::*Getter)() const, void (T::*Setter)(Type)>
        struct PropertyRW : PropertyR<T, Type, Getter>, PropertyW<T, Type, Setter>
        {
            using PropertyW < T, Type, Setter:: operator =;
        };

#define BEGIN_PROPERTIES union {
#define END_PROPERTIES };
#define PROPERTIES union

    }
}

#endif // PROPERTY_HPP
