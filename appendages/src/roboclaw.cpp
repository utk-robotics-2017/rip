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
			m_set_m1_speed(createCommand("kSetM1Speed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m2_speed(createCommand("kSetM2Speed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m1m2_speed(createCommand("kSetM1M2Speed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m1_speed_accel(createCommand("kSetM1SpeedAccel", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m2_speed_accel(createCommand("kSetM2SpeedAccel", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m1m2_speed_accel(createCommand("kSetM1M2SpeedAccel", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m1_speed_dist(createCommand("kSetM1SpeedDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m2_speed_dist(createCommand("kSetM2SpeedDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1m2_speed_dist(createCommand("kSetM1M2SpeedDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1_speed_accel_dist(createCommand("kSetM1SpeedAccelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m2_speed_accel_dist(createCommand("kSetM2SpeedAccelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1m2_speed_accel_dist(createCommand("kSetM1M2SpeedAccelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1_speed_accel_decel_dist(createCommand("kSetM1SpeedAccelDecelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m2_speed_accel_decel_dist(createCommand("kSetM2SpeedAccelDecelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1m2_speed_accel_decel_dist(createCommand("kSetM1M2SpeedAccelDecelDist", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m1_encoder(createCommand("kReadM1Encoder", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m2_encoder(createCommand("kReadM2Encoder", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m1m2_encoder(createCommand("kReadM1M2Encoder", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m1_encoder_result(createCommand("kReadM1EncoderResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_m2_encoder_result(createCommand("kReadM2EncoderResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_m1m2_encoder_result(createCommand("kReadM1M2EncoderResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_m1_encoder_speed(createCommand("kReadM1EncoderSpeed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m2_encoder_speed(createCommand("kReadM2EncoderSpeed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m1m2_encoder_speed(createCommand("kReadM1M2EncoderSpeed", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_read_m1_encoder_speed_result(createCommand("kReadM1EncoderSpeedResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_m2_encoder_speed_result(createCommand("kReadM2EncoderSpeedResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_read_m1m2_encoder_speed_result(createCommand("kReadM1M2EncoderSpeedResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_reset_encoders(createCommand("kResetEncoders", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_get_buffers(createCommand("kGetBuffers", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_get_buffers_result(createCommand("kGetBuffersResult", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::CharType>())),
			m_set_m1_duty(createCommand("kSetM1Duty", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>())),
			m_set_m2_duty(createCommand("kSetM2Duty", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>())),
			m_set_m1m2_duty(createCommand("kSetM1M2Duty", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>())),
			m_set_m1_velocity_pid(createCommand("kSetM2VelocityPID", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>())),
			m_set_m2_velocity_pid(createCommand("kSetM2VelocityPID", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>()))
		{
			if (config.find("address") == config.end())
			{
				throw AppendageMissingField(fmt::format("Roboclaw: missing config field 'address'"));
			}
			m_address = config["address"];
			if (config.find("ticks_per_rev") == config.end())
			{
				throw AppendageMissingField(fmt::format("Roboclaw: missing config field 'address'"));
			}
			m_ticks_per_rev = config.at("ticks_per_rev");
		}

		std::shared_ptr<Appendage> Roboclaw::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
		{
			return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Roboclaw>(new Roboclaw(config, command_map, device)));
		}

		void Roboclaw::setM1Speed(int32_t speed)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m1_speed, m_id,
					m_address, speed);
		}

		void Roboclaw::setM2Speed(int32_t speed)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m2_speed, m_id,
					m_address, speed);
		}

		void Roboclaw::setM1M2Speed(int32_t speed1, int32_t speed2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m1m2_speed, m_id,
					m_address, speed1, speed2);
		}

		void Roboclaw::setM1SpeedAccel(uint32_t accel, int32_t speed)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m1_speed_accel, m_id,
					m_address, accel, speed);
		}

		void Roboclaw::setM2SpeedAccel(uint32_t accel, int32_t speed)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m2_speed_accel, m_id,
					m_address, accel, speed);
		}

		void Roboclaw::setM1M2SpeedAccel(uint32_t accel, int32_t speed1, int32_t speed2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m1m2_speed_accel, m_id,
					m_address, accel, speed1, speed2);
		}

		void Roboclaw::setM1SpeedDist(int32_t speed, uint32_t distance, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1_speed_dist, m_id,
					m_address, speed, distance, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM2SpeedDist(int32_t speed, uint32_t distance, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m2_speed_dist, m_id,
					m_address, speed, distance, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM1M2SpeedDist(int32_t speed1, uint32_t distance1, int32_t speed2,
			uint32_t distance2, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1m2_speed_dist, m_id,
					m_address, speed1, distance1, speed2, distance2, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM1SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1_speed_accel_dist, m_id,
					m_address, accel, speed, distance, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM2SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m2_speed_accel_dist, m_id,
					m_address, accel, speed, distance, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM1M2SpeedAccelDist(uint32_t accel, int32_t speed1, uint32_t distance1,
			int32_t speed2, uint32_t distance2, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1m2_speed_accel_dist, m_id,
					m_address, accel, speed1, distance1, speed2, distance2, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM1SpeedAccelDecelDist(uint32_t accel, int32_t speed, uint32_t deccel,
			uint32_t position, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1_speed_accel_decel_dist, m_id,
					m_address, accel, speed, deccel, position, static_cast<uint8_t>(flag));
		}

		void Roboclaw::setM2SpeedAccelDecelDist(uint32_t accel, int32_t speed, uint32_t deccel,
			uint32_t position, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m2_speed_accel_decel_dist, m_id,
					m_address, accel, speed, deccel, position, static_cast<uint8_t>(flag));
		}

		/*void Roboclaw::setM1M2SpeedAccelDecelDist(uint32_t accel1, int32_t speed1,uint32_t deccel1,
				uint32_t position1, uint32_t accel2, int32_t speed2, uint32_t deccel2, uint32_t position2, bool flag)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_set_m1m2_speed_accel_decel_dist, m_id,
					m_address, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, static_cast<uint8_t>(flag));
		}*/

		int32_t Roboclaw::readM1Encoder()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m1_encoder, m_id, m_address);
			return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m1_encoder_result));
		}

		int32_t Roboclaw::readM2Encoder()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m2_encoder, m_id, m_address);
			return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m2_encoder_result));
		}

		std::tuple<int32_t, int32_t> Roboclaw::readM1M2Encoders()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m1m2_encoder, m_id, m_address);
			return messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m1m2_encoder_result);
		}

		int32_t Roboclaw::readM1EncoderSpeed()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m1_encoder_speed, m_id, m_address);
			return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m1_encoder_speed_result));
		}

		int32_t Roboclaw::readM2EncoderSpeed()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m2_encoder_speed, m_id, m_address);
			return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m2_encoder_speed_result));
		}

		std::tuple<int32_t, int32_t> Roboclaw::readM1M2EncoderSpeed()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_read_m1m2_encoder_speed, m_id, m_address);
			return messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_m1m2_encoder_speed_result);
		}

		void Roboclaw::resetEncoders()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_reset_encoders, m_id, m_address);
		}

		std::tuple<uint8_t, uint8_t> Roboclaw::getBuffers()
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_device, m_get_buffers, m_id, m_address);
			return messenger.receive<cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::CharType>(m_get_buffers_result);
		}

		void Roboclaw::setM1Duty(int16_t duty)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>(m_device, m_set_m1_duty, m_id,
					m_address, duty);
		}

		void Roboclaw::setM2Duty(int16_t duty)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>(m_device, m_set_m2_duty, m_id,
					m_address, duty);
		}

		void Roboclaw::setM1M2Duty(int16_t duty1, int16_t duty2)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedIntegerType>(m_device, m_set_m1m2_duty, m_id,
					m_address, duty1, duty2);
		}

		void Roboclaw::setM1VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m1_velocity_pid, m_id,
					m_address, Kp, Ki, Kd, qpps);
		}

		void Roboclaw::setM2VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps)
		{
			cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType,
				cmdmessenger::ArduinoCmdMessenger::CharType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::FloatType,
				cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_device, m_set_m2_velocity_pid, m_id,
					m_address, Kp, Ki, Kd, qpps);
		}

		units::Angle Roboclaw::getAngle(bool motor, bool continuous)
		{
			int32_t ticks;
			units::Angle rv;
			if(motor)
			{
				ticks = readM2Encoder();
			}
			else
			{
				ticks = readM1Encoder();
			}
			if(continuous)
			{
				ticks %= static_cast<int32_t>(m_ticks_per_rev);
			}
			rv = (ticks/m_ticks_per_rev) * 360 * units::deg;
			return rv;
		}

		void Roboclaw::setAngularSpeed(bool motor, units::AngularVelocity av)
		{
			if(motor)
			{
				setM2Speed(av.to(units::deg/units::s) / 360 * m_ticks_per_rev);
			}
			else
			{
				setM1Speed(av.to(units::deg/units::s) / 360 * m_ticks_per_rev);
			}
		}

		void Roboclaw::stop()
		{
			setM1M2Duty(0, 0);
		}

		bool Roboclaw::diagnostic()
		{
			int32_t ticks;

			std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
			misc::Logger::getInstance()->debug("Roboclaw appendage diagnostics start");
			//encoder reading diag
			misc::Logger::getInstance()->debug("Read encoders for 5s in a loop");
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			start_time = std::chrono::system_clock::now();
			//buffer reading diag
			misc::Logger::getInstance()->debug("Getting buffers for 2s in a loop");
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 2000)
			{
				misc::Logger::getInstance()->debug(fmt::format("Buffer M1: {} M2: {}", std::get<0>(getBuffers()), std::get<1>(getBuffers())));
			}
			start_time = std::chrono::system_clock::now();
			//encoder resetting diag
			misc::Logger::getInstance()->debug("Resetting encoders for 2s in a loop");
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 2000)
			{
				resetEncoders();
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
			}
			start_time = std::chrono::system_clock::now();
			//duty setting diag
			misc::Logger::getInstance()->debug("Setting Duty to ~1/2 Power, forward for 5 seconds");

			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1M2Duty(16000, 16000);
				setM1Duty(16000);
				setM2Duty(16000);
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			//speed accel drive diag
			misc::Logger::getInstance()->debug("Setting speed accel drive (5s)");
			start_time = std::chrono::system_clock::now();

			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1M2SpeedAccel(12000, 12000, 12000);
				setM1SpeedAccel(12000, 12000);
				setM2SpeedAccel(12000, 12000);
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			misc::Logger::getInstance()->debug("Setting speed drive (5s)");
			start_time = std::chrono::system_clock::now();
			//speed drive diag
			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1Speed(12000);
				setM2Speed(12000);
				setM1M2Speed(12000, 12000);
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			//speed dist drive diag
			misc::Logger::getInstance()->debug("Setting speed dist drive (5s)");
			start_time = std::chrono::system_clock::now();

			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1SpeedDist(12000, 12000);
				setM2SpeedDist(12000, 12000);
				setM1M2SpeedDist(12000, 12000, 12000, 12000);
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			//speed accel dist drive diag
			misc::Logger::getInstance()->debug("Setting speed accel dist drive (5s)");
			start_time = std::chrono::system_clock::now();

			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1SpeedAccelDist(12000, 12000, 12000);
				setM2SpeedAccelDist(12000, 12000, 12000);
				setM1M2SpeedAccelDist(12000, 12000, 12000, 12000, 12000);
				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			/*
			//speed accel decel dist drive diag
			misc::Logger::getInstance()->debug("Setting speed accel decel dist drive (5s)");
			start_time = std::chrono::system_clock::now();

			while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
			{
				setM1SpeedAccelDecelDist(12000, 12000, 12000, 12000);
				setM2SpeedAccelDecelDist(12000, 12000, 12000, 12000);

				misc::Logger::getInstance()->debug(fmt::format("Raw readings: 1: {}  | 2: {}", readM1Encoder(), readM2Encoder()));
				misc::Logger::getInstance()->debug(fmt::format("Encoder velocity: M1 {} M2 {}", readM1EncoderSpeed(), readM2EncoderSpeed()));
			}
			stop();
			*/

			misc::Logger::getInstance()->debug("Roboclaw appendage diagnostics complete");

			return 1;
		}
	}
}
