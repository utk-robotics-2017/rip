#ifndef APPENDAGE_HPP
#define APPENDAGE_HPP

namespace rip
{
    /**
     * @class Appendage
     * @brief The base class for appendages.
     */
    class Appendage
    {
        public:
            /**
             * @brief Returns the label for the the appendage
             *
             * @returns The label for the appendage
             */
            std::string getLabel() const;

            /**
             * @brief Returns the type of the appendage
             *
             * @returns The type of the appendage
             */
            std::string getType() const;

        protected:
            /**
             * @brief Constructor
             *
             * @note Is protected so that it cannot be used directly. The AppendageFactory must be used
             */
            Appendage();

        private:

            std::string m_label;
            std::string m_type;
    };
}

#endif // APPENDAGE_HPP
