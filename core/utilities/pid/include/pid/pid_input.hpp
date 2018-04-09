#ifndef PID_INPUT_HPP
#define PID_INPUT_HPP

namespace rip
{
    namespace pid
    {
        /**
         * @class PidInput
         *
         * An interface for a generic sensor input to a PID
         *
         * All sensors that can be used with the PID will derive
         * from this class.
         */
        class PidInput
        {
        public:
            enum class Type
            {
                kDisplacement,
                kRate
            };


            virtual void setType(Type type);
            virtual Type type();
            virtual double get() = 0;

        private:
            Type m_type = Type::kDisplacement;
        };
    }
}

#endif // PID_INPUT_HPP
