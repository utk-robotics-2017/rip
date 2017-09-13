#ifndef STATUS_SERVER_HPP
#define STATUS_SERVER_HPP

#include <string>
#include <memory>
#include <map>
#include <threading>

#include <json.hpp>

namespace rip
{
    namespace status
    {
        class StatusServer
        {
        public:
            /**
             *
             */
            static std::shared_ptr<StatusServer> getInstance();

            /**
             * Starts the status server
             */
            void start();

            /**
             * Puts a value for a specific key for a specific appendage
             */
            template<typename T>
            void put(const std::string& type, const std::string& label, const std::string& key, T value)
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_data[type][label][key] = value;
                m_cv.notify_one();
            }

            /**
             *
             */
            template<typename T>
            T get(const std::string& type, const std::string& label, const std::string& key)
            {
                if (m_data.find(type) == m_data.end() || m_data[type].find(label) == m_data[type].end() || m_data[type][label].find(key) == m_data[type][label].end())
                {
                    // todo(Andrew): throw exception
                }
                return m_data[type][label][key].get<T>();
            }

        private:
            /**
             * Constructor
             */
            StatusServer();

            /**
             * Destructor
             */
            ~StatusServer();

            /**
             * Runs the status server
             */
            void run();

            static std::shared_ptr<StatusServer> m_singleton;

            std::unique_ptr<std::thread> m_thread;
            std::condition_variable m_cv;
            std::mutex m_mutex;

            nlohmann::json m_data;
            nlohmann::json m_last_sync;
        };
    }
}
#endif // STATUS_SERVER_HPP