#ifndef ROBOCLAW_HPP
#define ROBOCLAW_HPP

#include <tuple>
#include <misc/logger.hpp>
#include <units/units.hpp>
#include <fmt/format.h>
#include <cmd_messenger/command.hpp>
#include <motor_controllers/motor_dynamics.hpp>
#include "appendages/appendage.hpp"
#include "appendages/appendage_factory.hpp"
#include <json.hpp>

namespace rip
{
	namespace appendages
	{
		class Roboclaw : public Appendage
		{
		public:
			void setM1Speed(int32_t speed);
			void setM2Speed(int32_t speed);
			void setM1M2Speed(int32_t speed);

			void setM1SpeedAccel(uint32_t accel, int32_t speed);
			void setM2SpeedAccel(uint32_t accel, int32_t speed);
			void setM1M2SpeedAccel(uint32_t accel, int32_t speed1, int32_t speed2);

			void setM1SpeedDist(int32_t speed, uint32_t distance);
			void setM2SpeedDist(int32_t speed, uint32_t distance);
			void setM1M2SpeedDist(int32_t speed1, uint32_t distance1, int32_t speed2, uint32_t distance2);

			void setM1SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance);
			void setM2SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance);
			void setM1M2SpeedAccelDist(uint32_t accel, int32_t speed1, uint32_t distance1, int32_t speed2, uint32_t distance2);

			void setM1SpeedAccelDecelDist(uint32_t accel, int32_t speed,uint32_t deccel,uint32_t position);
			void setM2SpeedAccelDecelDist(uint32_t accel, int32_t speed,uint32_t deccel,uint32_t position);
			void setM1M2SpeedAccelDecelDist(uint32_t accel1, int32_t speed1,uint32_t deccel1,
				uint32_t position1, uint32_t accel2, int32_t speed2, uint32_t deccel2, uint32_t position2);

			int32_t readM1Encoder();
			int32_t readM2Encoder();
			std::array<int32_t, 2> readM1M2Encoders();
			std::array<units::Distance, 2> readEncoders(); //Not implemented on arduino
			units::Distance readEncoder(bool motor); //Not implemented on arduino

			int32_t readM1EncoderSpeed();
			int32_t readM2EncoderSpeed();
			std::array<int32_t, 2> readM1M2EncoderSpeed();
			std::array<units::Velocity, 2> readEncoderSpeeds(); //Not implemented on arduino
			units::Velocity readEncoderSpeed(bool motor); //Not implemented on arduino

			void setM1Duty(int16_t duty);
			void setM2Duty(int16_t duty);
			void setDuties(int16_t duty1, int16_t duty2); //Not implemented on arduino
			void setDuty(bool motor, int16_t duty); //Not implemented on arduino

			void setVelocityPID(bool motor, float Kp, float Ki, float Kd, uint32_t qpps);
			std::array<uint8_t, 2> getBuffers();
			void resetEncoders();

			void setDynamics(bool motor, const MotorDynamics& dynamics, bool respectBuffer=true); //Not implemented on arduino
			void setDynamics(const MotorDynamics& dynamics, bool respectBuffer=true); //Not implemented on arduino
			/**
			* Stop! ^0^
			*/
			virtual void stop() override;

			virtual bool diagnostic() override;

		protected:
			friend class AppendageFactory;

			/**
			* Function wrapper for the constructor so it can be pointed to
			*
			* @param config The config from arduino gen
			* @param command_map A map of the name of the commands to their enumerations
			* @param device The connection to the device
			*/
			static std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

		private:
			/**
			* Constructor
			*
			* @param config The config from arduino gen
			* @param command_map A map of the name of the commands to their enumerations
			* @param device The connection to the device
			*/
			Roboclaw(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

			uint8_t m_address;
			double m_ticks_per_rev;
			units::Distance m_wheel_radius;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_speed;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_speed;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_speed;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_speed_accel;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_speed_accel;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_speed_accel;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_speed_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_speed_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_speed_dist;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_speed_accel_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_speed_accel_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_speed_accel_dist;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_speed_accel_decel_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_speed_accel_decel_dist;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_speed_accel_decel_dist;

			std::shared_ptr<cmdmessenger::Command> m_read_m1_encoder;
			std::shared_ptr<cmdmessenger::Command> m_read_m2_encoder;
			std::shared_ptr<cmdmessenger::Command> m_read_m1m2_encoder;
			std::shared_ptr<cmdmessenger::Command> m_read_encoder_result;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders_result;

			std::shared_ptr<cmdmessenger::Command> m_reset_encoders;
			std::shared_ptr<cmdmessenger::Command> m_get_buffers;
			std::shared_ptr<cmdmessenger::Command> m_get_buffers_result;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_duty;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_duty;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_duty;

			std::shared_ptr<cmdmessenger::Command> m_set_speed_pid;

			std::shared_ptr<cmdmessenger::Command> m_read_m1_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_m2_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_m1m2_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_encoder_speed_result;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders_speed_result;

		};
	}
}

#endif // ROBOCLAW_HPP
