/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <memory>
#include <json.hpp>

namespace rip
{
    namespace misc
    {
        struct constants
        {
            static const char* kArduinoGenHome;
            static const char* kLoggerName;
            static const char* kPathFollowingMaxAccel;
            static const char* kSegmentCompletionTolerance;

            static std::shared_ptr<constants> getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::make_shared<constants>();
                }
                return m_singleton;
            }

            void load(const nlohmann::json& constants)
            {
                m_constants = constants;
            }

            template <typename T>
            T get(const std::string& name)
            {
                return m_constants[name].get<T>();
            }

            template <typename T>
            void set(const std::string& name, T value)
            {
                m_constants[name] = value;
            }

        private:
            static std::shared_ptr<constants> m_singleton;
            nlohmann::json m_constants;

        };
    }
}

#endif // CONSTANTS_HPP
