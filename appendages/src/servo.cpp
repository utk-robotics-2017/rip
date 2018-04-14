#include "appendages/servo.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

using namespace rip::utilities;

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
            , m_detach(
              createCommand(
                "kDetachServo",
                command_map,
                cmdmessenger::ArduinoCmdMessenger::makeArgumentString<
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
            detach();
        }

        void Servo::detach()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send(m_device, m_detach, m_id);
        }

        bool Servo::diagnostic()
        {
          int new_val = 0;
          std::cout << " === DIAG CONTROLS: === " << '\n';
          std::cout << " '-1': Quit." << '\n';
          std::cout << " '-2': Detach servo." << '\n';
          std::cout << " '0-180': Set servo position." << '\n';
          while (new_val != -1) {
            // write(0);
            std::cout << " >>> Please enter a servo value (int) to write (-1 quits): ";
            std::cin >> new_val;
            std::cout << " >>> Working...\n";
            if (new_val == -1) break;
            else if (new_val == -2)
            {
              detach();
            }
            else
            {
              write(new_val);
            }
          }
          return true;
        }

        std::shared_ptr<Appendage> Servo::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Servo>(new Servo(config, command_map, device)));
        }
    }
}
