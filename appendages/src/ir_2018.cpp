#include "appendages/ir_2018.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        Ir2018::Ir2018(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kReadIr2018", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_read_result(createCommand("kReadIr2018Result", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType>()))
            , m_calibrate(createCommand("kCalibrateIr2018", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_calibrate_result(createCommand("kCalibrateIr2018Result", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::BooleanType>()))
        {
        }

        std::array<bool, 4> Ir2018::read()
        {
            std::array<bool, 4> rv;

            cmdmessenger::ArduinoCmdMessenger messenger;
            try
            {
                messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_read, m_id);
                char data = std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::CharType>(m_read_result));
                rv[0] = true;
                rv[1] = data & 1 << 0;
                rv[2] = data & 1 << 1;
                rv[3] = data & 1 << 2;
            }
            catch ()
            {
                rv[0] = false;
            }
            return rv;
        }

        bool Ir2018::calibrate()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_calibrate, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::BooleanType>(m_calibrate_result));
        }

        void Ir2018::stop()
        {

        }

        bool Ir2018::diagnostic()
        {
            // todo
            return true;
        }

        std::shared_ptr<Appendage> Ir2018::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<DigitalInput>(new Ir2018(config, command_map, device)));
        }
    }
}
