#include "status_server.hpp"

namespace rip
{
    namespace status
    {
        std::shared_ptr<StatusServer> StatusServer::m_singleton = std::shared_ptr(nullptr);

        StatusServer::getInstance()
        {
            if (!m_singleton)
            {
                m_singleton = std::make_shared<StatusServer>();
            }
            return m_singleton;
        }

        StatusServer::StatusServer()
            : m_thread(nullptr)
        {

        }

        StatusServer::~StatusServer()
        {
            m_running = false;
        }

        void StatusServer::start()
        {
            m_thread = std::unique_ptr<std::thread>(run);
        }

        void StatusServer::run()
        {
            while (m_running)
            {
                m_cv.wait(m_mutex);
                json diff = json::diff(m_data, m_last_sync);
                send(diff);
            }
        }
    } // status
} // rip