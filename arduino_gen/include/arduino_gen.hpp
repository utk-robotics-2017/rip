#ifndef ARDUINO_GEN_H
#define ARDUINO_GEN_H

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace rip
{
    namespace arduinogen
    {
        class Appendage;
        class AppendageTemplate;

        /**
         * @class ArduinoGen
         * @brief A generator for Arduino Code that communicates with RIP
         */
        class ArduinoGen
        {
        public:
            /**
             * @brief Constructor
             */
            ArduinoGen(std::string arduino, std::string parent_folder);

            /**
             * @brief Reads the config file with the appendages
             *
             * @param filepath The filepath to the appendages config file to read
             * @param copy Whether to copy the appendages config file into the output folder
             */
            void readConfig(std::string filepath, bool copy = true);

            /**
             * @brief Writes the Arduino code, build script, and the config file used by RIP
             */
            void generateOutput();

        private:
            /**
             * @brief Removes the device folder if it exists and creates a new one
             */
            void setupFolder();

            /**
             * TODO: Properly comment
             */
            void fillCodeTemplate();

            /**
             * TODO: Properly comment
             */
            void loadTemplates();

            /**
             * TODO: Properly comment
             */
            void replace(std::string& base, const std::string& replacee, const std::string& replacer);

            /**
             * @brief Returns the header for the Arduino Code
             * @returns The header for the Arduino Code
             */
            std::string getHeader();

            /**
             * @brief Returns the includes for the Arduino Code
             * @returns The includes for the Arduino Code
             */
            std::string getIncludes();

            /**
             * @brief Returns the code for the constructors
             * @returns The code of the constructors
             */
            std::string getConstructors();

            /**
             * @brief Returns the code of the setup function
             * @returns The code for the setup function
             */
            std::string getSetup();

            /**
             * @brief Returns the code for the loop function
             * @returns The code for the loop function
             */
            std::string getLoop();

            /**
             * @brief Returns the code for all the command enums
             * @returns The code for all the command enums
             */
            std::string getCommandEnums();

            /**
             * @brief Returns the code for attaching enums to the callback functions
             * @returns The code for attaching enums to the callback functions
             */
            std::string getCommandAttaches();

            /**
             * @brief Returns the code for the command callback functions
             * @returns The code for the command callback functions
             */
            std::string getCommandCallbacks();

            /**
             * @brief Returns the code for all the extra functions
             * @returns The code for all the extra functions
             */
            std::string getExtras();

            /**
             * TODO: Properly comment
             */
            std::string getCoreConfig();

            /**
             * TODO: Properly comment
             */
            std::string getUploadScript();

            std::string m_arduino;
            std::string m_parent_folder;

            std::string m_code;

            std::map<std::string, int> m_commands;

            std::multimap< std::string, std::shared_ptr<Appendage> > m_appendages;

            std::vector<AppendageTemplate> m_appendage_templates;
        };
    }
}

#endif // ARDUINOCODEGENERATOR_H
