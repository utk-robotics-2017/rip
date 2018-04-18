#ifndef DIAG_HPP
#define DIAG_HPP

namespace rip
{
    namespace diag
    {
        class Diag
        {
        public:
            Diag(std::shared_ptr<framework::RobotBase> robot)
                : m_robot(robot)
            {}

            std::vector<std::string> subsystemNames()
            {
                std::vector<std::string> rv;
                for (auto const& element : m_robot->m_subsystems)
                {
                    rv.push_back(element.first);
                }
                return rv;
            }

            std::vector<std::string> appendageNames()
            {
                std::vector<std::string> rv;
                for (auto const& element : m_robot->m_spine->m_appendages)
                {
                    rv.push_back(element.first);
                }
                return rv;
            }

            void runAll()
            {
                for (auto const& element : m_robot->m_subsystems)
                {
                    element.second->diagnostic();
                }
            }

            bool hasSubsystem(const std::string& name)
            {
                if (m_robot->m_subsystems.find(name) != m_robot->m_subsystems.end())
                {
                    return true;
                }
                return false;
            }

            void runSubsystem(const std::string& name)
            {
                if (m_robot->m_subsystems.find(name) != m_robot->m_subsystems.end())
                {
                    m_robot->m_subsystems[name]->diagnostic();
                }

                // throw error
            }

            bool hasAppendage(const std::string& name)
            {
                return m_robot->m_spine->has(name);
            }

            bool runAppendage(const std::string& name)
            {
                if (m_robot->m_spine->has(name))
                {
                    return m_robot->m_spine->get(name)->diagnostic();
                }
                else
                {
                    return false;
                }
            }

        private:
            std::shared_ptr<framework::RobotBase> m_robot;
        };
    }
}

#endif // DIAG_HPP
