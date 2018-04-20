#include "framework/robot_base.hpp"

#include <istream>
#include <ostream>
#include <cppfs/FileHandle.h>
#include <cppfs/fs.h>
#include <framework/exceptions.hpp>
#include <misc/logger.hpp>
#include <spdlog/spdlog.h>

namespace rip
{
    namespace framework
    {
        RobotBase::RobotBase(const std::string& config)
            : m_running(false)
            , m_config_path(config)
        {
        }

        RobotBase::~RobotBase()
        {
            m_running = false;
        }

        void RobotBase::init()
        {
            misc::Logger::getInstance()->debug("Robot is initializing...");
            cppfs::FileHandle config_file = cppfs::fs::open(m_config_path);
            if (!config_file.exists())
            {
                throw FileNotFound();
            }

            std::unique_ptr<std::istream> in = config_file.createInputStream();
            nlohmann::json j;
            (*in) >> j;

            if(j.find("constants") != j.end())
            {
                misc::constants::load(j["constants"]);
            }


            std::vector<std::string> devices;
            if(j.find("devices") != j.end())
            {
                for (nlohmann::json d : j["devices"])
                {
                    std::string device_name = d;
                    devices.push_back(device_name);
                }
            }
            m_spine = std::unique_ptr<Spine>(new Spine);


            if(j.find("arduino_gen_home") != j.end())
            {
                misc::Logger::getInstance()->debug("Loading spine appendages...");
                m_spine->loadDevices(j["arduino_gen_home"], devices);
            }
            else
            {
                misc::Logger::getInstance()->error("Arduino Gen Home not found in config");
                throw AppendageNotFound();
            }

            if(j.find("subsystems") != j.end())
            {
                createSubsystems(j["subsystems"]);
            }
            else
            {
                misc::Logger::getInstance()->error("Subsystems not found in config");
                throw SubSystemsNotFound();
            }

            if(j.find("actions") != j.end())
            {
                createRoutine(j["actions"]);
            }
            else
            {
                misc::Logger::getInstance()->error("Actions not found in config");
                throw ActionsNotFound();
            }

            if(j.find("state_file") != j.end())
            {
                m_state_file = cppfs::fs::open(j["state_file"]).createOutputStream();
            }
        }

        void RobotBase::start()
        {
            m_running = true;
            misc::Logger::getInstance()->debug("Starting the robot...");
            run();
        }

        void RobotBase::stop()
        {
            misc::Logger::getInstance()->debug("Stopping the robot...");
            m_running = false;
            misc::Logger::getInstance()->debug("Stopping subsystems");
            for(auto subsystem : m_subsystems)
            {
                misc::Logger::getInstance()->debug("Stopping {}", subsystem.first);
                subsystem.second->stop();
            }
            m_spine->stop();
        }

        void RobotBase::run()
        {
            nlohmann::json state;
            misc::Logger::getInstance()->debug("run");

            // Loop through the routine
            for (std::shared_ptr<Action> action : m_routine)
            {
                misc::Logger::getInstance()->debug("loop");

                // If ever interrupted then stop
                if (!m_running)
                {
                    break;
                }

                // Setup the action
                misc::Logger::getInstance()->debug("Setting up action: {}", action->name());
                action->setup(state);

                /*
                // Reset the state file
                if(m_state_file != nullptr)
                {
                    m_state_file->clear();
                    (*m_state_file) << state;
                    m_state_file->flush();
                }
                */

                // If ever interrupted then stop
                if (!m_running)
                {
                    break;
                }

                // Update until the action is finished
                while (m_running && !action->isFinished())
                {
                    action->update(state);

                    /*
                    // Reset the state file
                    if(m_state_file != nullptr)
                    {
                        m_state_file->clear();
                        (*m_state_file) << state;
                        m_state_file->flush();
                    }
                     */

                    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_update_time.to(units::ms))));
                }

                // If ever interrupted then stop
                if (!m_running)
                {
                    break;
                }

                // Teardown the action
                misc::Logger::getInstance()->debug("Tearing down action: {}", action->name());
                action->teardown(state);

                /*
                // Reset the state file
                if(m_state_file != nullptr)
                {
                    m_state_file->clear();
                    (*m_state_file) << state;
                    m_state_file->flush();
                }
                 */
            }

            misc::Logger::getInstance()->debug("Stopping subsystems");
            for(auto subsystem : m_subsystems)
            {
                misc::Logger::getInstance()->debug("Stopping {}", subsystem.first);
                subsystem.second->stop();
            }
            m_spine->stop();

            m_running = false;
        }
    }
}
