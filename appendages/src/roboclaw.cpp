#include "appendages/roboclaw.hpp"
#include <ctime>
#include <utility>
#include <tuple>
#include <memory>
#include <chrono>
#include <thread>
#include <cmd_messenger/cmd_messenger.hpp>

#include "appendages/exceptions.hpp"

namespace rip
{
	namespace appendages
	{
		Roboclaw::Roboclaw(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
			: Appendage(config, device),
			m_set_speed(createCommand("kSetSpeed", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_speed_accel(createCommand("kSetSpeedAccel", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_encoders(createCommand("kReadEncoders", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_encoders_result(createCommand("kReadEncodersResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_duty(createCommand("kSetDuty", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType, cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>())),
			m_set_velocity_pid(createCommand("kSetVelocityPID", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>()))
		{
			if (config.find("address") == config.end())
			{
				throw AppendageMissingField(fmt::format("appendage roboclaw missing address"));
			}
			m_address = config["address"];
		}

		std::shared_ptr<Appendage> Roboclaw::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
		{
			return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Roboclaw>(new Roboclaw(config, command_map, device)));
		}

		void Roboclaw::SetSpeed(int32_t speed1, int32_t speed2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_speed, m_address, speed1, speed2);
		}

		void Roboclaw::SetSpeedAccel(int32_t accel, int32_t speed1, int32_t speed2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_speed_accel, m_address, accel, speed1, speed2);
		}

		std::tuple<uint32_t, uint32_t> Roboclaw::ReadEncoders()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_encoders, m_address);
			return messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_encoders_result);
		}

		void Roboclaw::SetDuty(int16_t duty1, int16_t duty2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType, cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>(m_device, m_set_duty, m_address, duty1, duty2);
		}

		void Roboclaw::SetVelocityPID(uint8_t motor, float Kp, float Ki, float Kd, uint32_t qpps)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::CharType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::FloatType, cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_velocity_pid, m_address, motor, Kp, Ki, Kd, qpps);
		}

		void Roboclaw::stop() {
			SetSpeed(0, 0);
		}

		bool Roboclaw::diagnostic()
		{
			std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
			misc::Logger::getInstance()->debug("Roboclaw appendage diagnostics start");

			misc::Logger::getInstance()->debug("Read encoders for 10s in a loop");
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 10000)
			{
				misc::Logger::getInstance()->debug(fmt::format("Encoder 1 Ticks: {} | Encoder 2 Ticks: ", std::get<0>(ReadEncoders()), std::get<0>(ReadEncoders())));
			}
			misc::Logger::getInstance()->debug("Setting Duty to ~1/2 Power, forward for 5 seconds");
			SetDuty(16000, 16000);
			start_time = std::chrono::system_clock::now();
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{}
			stop();
			misc::Logger::getInstance()->debug("Setting speed accel drive (5s)");
			SetSpeedAccel(12000, 12000, 12000);
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{}
			stop();
		}
	}
}
