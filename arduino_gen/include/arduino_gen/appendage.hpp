#ifndef APPENDAGE_H
#define APPENDAGE_H

#include <string>
#include <map>
#include <memory>

#include <json.hpp>

namespace rip
{
    namespace arduinogen
    {
        class Appendage
        {
        public:
            /**
             * @brief Default Constructor
             */
            Appendage() = default;

            /**
             * @brief Constructor that loads from a portion of json
             * @param data The data to load for this appendage
             * @param appendages The list of appendages for if this is a derivative appendage
             * @param test Whether to test the type or not (default: true)
             */
            Appendage(nlohmann::json data,
                      std::multimap<std::string, std::shared_ptr<Appendage>>& appendages,
                      std::string appendage_data_folder = "appendages", bool test = true);

            /**
             * @brief Returns the label for this appendage
             */
            std::string getLabel() const;

            /**
             * @brief Returns the type for this appendage
             */
            std::string getType() const;

            bool has(std::string data_name) const;

            /**
             * @brief Templated function that gets a values from the json
             *
             * @param data_name The name of the value
             *
             * @returns A value from this appendage
             *
             * @typename T The type of the value that should be returned (int, float, string)
             * @note This function must remain in the header
             */
            template <typename T>
            T get(std::string data_name) const
            {
                return m_data[data_name].get<T>();
            }

            /**
             * @brief Returns the string to use in the Arduino code for the given value
             *
             * @param  data_name The name of the value
             *
             * @returns The Arduino code for the given value
             */
            std::string getString(std::string data_name) const;

            /**
             * Check if the data is the specifed type
             * @param  data_name The name of the value
             * @param type The type to check against
             * @return Whether the data is the specified type
             */
            bool isType(std::string data_name, std::string type) const;

            /*
             * TODO: Properly document
             */
            nlohmann::json getCoreJson(int index) const;

        private:
            /**
             * @brief Tests whether the type has all the parameters specified in a json file.
             *
             * @exception TemplateNotFound This exception is thrown if a file cannot be found
             *            for this type.
             *
             * @exception ParameterInvalid This exception is thrown if a parameter is missing,
             *            included but not in the template, or has the wrong type.
             */
            void testType() const;

            nlohmann::json m_data;

            static std::map< std::string, std::map< std::string, std::string > > m_type_cache;

            std::string m_appendage_data_folder;
        };
    }
}

#endif // APPENDAGE_H
