#include "appendages/servo.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        Servo::Servo(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_write(
              createCommand(
                "kSetServo",
                command_map,
                cmdmessenger::ArduinoCmdMessenger::makeArgumentString<
                  cmdmessenger::ArduinoCmdMessenger::IntegerType,
                  cmdmessenger::ArduinoCmdMessenger::IntegerType>()
                )
              )
        {
        }

        void Servo::write(int value)
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<int>(m_device, m_write, m_id, value);
        }

        void Servo::stop()
        {

        }

        bool Servo::diagnostic()
        {
          int new_val = 0;
          while (new_val != -1) {
            // write(0);
            std::cout << " >>> Please enter a servo value (int) to write (-1 quits): ";
            std::cin >> new_val;
            if (new_val == -1) break;
            std::cout << " >>> Writing...\n";
            write(new_val);
          }
          return true;
        }

        std::shared_ptr<Appendage> Servo::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Servo>(new Servo(config, command_map, device)));
        }
    }
}
