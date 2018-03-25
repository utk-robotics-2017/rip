#ifndef ROBOCLAW_HPP
#define ROBOCLAW_HPP

#include <tuple>
#include <misc/logger.hpp>
#include <units/units.hpp>
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
			void drive(bool motor, int32_t speed)
			void drive(int32_t speed1, int32_t speed2);
			//void setSpeedAccel(int32_t accel, int32_t speed1, int32_t speed2);
			std::tuple<units::Distance, units::Distance> readEncoders();
			units::Distance readEncoder(bool motor)
			std::tuple<units::Velocity, units::Velocity> readEncoderSpeeds();
			//void setDuty(int16_t duty1, int16_t duty2);
			void setVelocityPID(bool motor, float Kp, float Ki, float Kd, uint32_t qpps);
			void setDynamics(bool motor, const MotorDynamics& dynamics, bool respectBuffer=true);
			void setDynamics(const MotorDynamics& dynamics, bool respectBuffer=true);
			void resetEncoders();
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

			std::shared_ptr<cmdmessenger::Command> m_set_speed;
			std::shared_ptr<cmdmessenger::Command> m_set_speed_accel;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders_result;
			std::shared_ptr<cmdmessenger::Command> m_set_duty;
			std::shared_ptr<cmdmessenger::Command> m_set_velocity_pid;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders_velocity;
			std::shared_ptr<cmdmessenger::Command> m_read_encoders_velocity_result;

		};
	}
}

#endif // ROBOCLAW_HPP
