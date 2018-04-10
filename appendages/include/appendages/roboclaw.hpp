#ifndef ROBOCLAW_APPENDAGE_HPP
#define ROBOCLAW_APPENDAGE_HPP

#include <tuple>
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <cmd_messenger/command.hpp>
#include "appendages/appendage.hpp"
#include "appendages/appendage_factory.hpp"
#include <json.hpp>
#include <units/units.hpp>

namespace rip
{
	namespace appendages
	{
		class Roboclaw : public Appendage
		{
		public:
			/**
			 * @brief PID based drive with speed for motor 1
			 * @param speed ticks / second
			 */
			void setM1Speed(int32_t speed);
			/**
			 * @brief PID based drive with speed for motor 2
			 * @param speed ticks / second
			 */
			void setM2Speed(int32_t speed);
			/**
			 * @brief PID based drive with speed for motors 1 and 2
			 * @param speed1 ticks / second
			 * @param speed2 ticks / second
			 */
			void setM1M2Speed(int32_t speed1, int32_t speed2);
			/**
			 * @brief PID based drive with speed, accel for motor 1
			 * @param accel ticks / second^2
			 * @param speed ticks / second
			 */
			void setM1SpeedAccel(uint32_t accel, int32_t speed);
			/**
			 * @brief PID based drive with speed, accel for motor 2
			 * @param accel ticks / second^2
			 * @param speed ticks / second
			 */
			void setM2SpeedAccel(uint32_t accel, int32_t speed);
			/**
			 * @brief PID based drive with speed, accel for motors 1&2
			 * @param accel  ticks / second^2
			 * @param speed1 ticks / second
			 * @param speed2 ticks / second
			 */
			void setM1M2SpeedAccel(uint32_t accel, int32_t speed1, int32_t speed2);
			/**
			 * @brief PID based drive with speed, dist for motor 1
			 * @param speed    ticks / second
			 * @param distance ticks
			 * @param flag     1 to apply instantly, 0 to buffer command
			 */
			void setM1SpeedDist(int32_t speed, uint32_t distance, bool flag=1);
			/**
			 * @brief PID based drive with speed, dist for motor 2
			 * @param speed    ticks / second
			 * @param distance ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM2SpeedDist(int32_t speed, uint32_t distance, bool flag=1);
			/**
			 * @brief PID based drive with speed, dist for motors 1&2
			 * @param speed1    ticks / second
			 * @param distance1 ticks
			 * @param speed2    ticks / second
			 * @param distance2 ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM1M2SpeedDist(int32_t speed1, uint32_t distance1, int32_t speed2, uint32_t distance2, bool flag=1);
			/**
			 * @brief PID based drive with speed, accel, dist for motor 1
			 * @param accel    ticks / second^2
			 * @param speed    ticks / second
			 * @param distance ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM1SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance, bool flag=1);
			/**
			 * @brief PID based drive with speed, accel, dist for motor 2
			 * @param accel    ticks / second^2
			 * @param speed    ticks / second
			 * @param distance ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM2SpeedAccelDist(uint32_t accel, int32_t speed, uint32_t distance, bool flag=1);
			/**
			 * @brief PID based drive with speed, accel, dist for motors 1&2
			 * @param accel     ticks / second^2
			 * @param speed1    ticks / second
			 * @param distance1 ticks
			 * @param speed2    ticks / second
			 * @param distance2 ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM1M2SpeedAccelDist(uint32_t accel, int32_t speed1, uint32_t distance1, int32_t speed2, uint32_t distance2, bool flag=1);
			/**
			 * @brief PID based drive with speed, accel, decel, dist for motor 1
			 * @param accel    ticks / second^2
			 * @param speed    ticks / second
			 * @param deccel   ticks / second^2
			 * @param position ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM1SpeedAccelDecelDist(uint32_t accel, int32_t speed, uint32_t deccel, uint32_t position, bool flag=1);
			/**
			 * @brief PID based drive with speed, accel, decel, dist for motors 2
			 * @param accel    ticks / second^2
			 * @param speed    ticks / second
			 * @param deccel   ticks / second^2
			 * @param position ticks
			 * @param flag      1 to apply instantly, 0 to buffer command
			 */
			void setM2SpeedAccelDecelDist(uint32_t accel, int32_t speed,uint32_t deccel,uint32_t position, bool flag=1);
			/*void setM1M2SpeedAccelDecelDist(uint32_t accel1, int32_t speed1,uint32_t deccel1,
				uint32_t position1, uint32_t accel2, int32_t speed2, uint32_t deccel2, uint32_t position2, bool flag=1);
			*/
			/**
			 * @brief Reads the tick count from motor 1
			 * @return ticks
			 */
			int32_t readM1Encoder();
			/**
			 * @brief Reads the tick count from motor 2
			 * @return ticks
			 */
			int32_t readM2Encoder();
			/**
			 * @brief Reads the tick count from motors 1&2
			 * @return ticks
			 */
			std::tuple<int32_t, int32_t> readM1M2Encoders();
			/**
			 * @brief Reads encoder ticks/s for motor M1.
			 * @return Motor M1 ticks/ second
			 */
			int32_t readM1EncoderSpeed();
			/**
			 * @brief Reads encoder ticks per second for motor M1.
			 * @return Motor m2 ticks/ second
			 */
			int32_t readM2EncoderSpeed();
			/**
			 * @brief Reads encoder ticks/s for motors M1&M2.
			 * @return tuple containing int32_t for m1 & m2 respectively
			 */
			std::tuple<int32_t, int32_t> readM1M2EncoderSpeed();
			/**
			 * @brief Resets encoders tick values.
			 */
			void resetEncoders();
			/**
			 * @brief: reads the number of commands in the buffer for
			 both motors, returns the buffer for just the specified motor.
			 Send: [Address, 47]
			 Receive: [BufferM1, BufferM2, CRC(2 bytes)]

			 @returns Buffer length for motors 1 or 2
			 *
			 */
			std::tuple<uint8_t, uint8_t> getBuffers();
			/**
			 * @brief Sets the duty for motor M1.
			 * @param duty (-32727 - 32726) for motor 1
			 */
			void setM1Duty(int16_t duty);
			/**
			 * @brief Sets duty for motor m2.
			 * @param duty (-32727 - 32726) for motor 2
			 */
			void setM2Duty(int16_t duty);
			/**
			 * @brief sets duty for motors m1 and m2
			 * @param duty1 (-32727 - 32726) for motor 1
			 * @param duty2 (-32727 - 32726) for motor 2
			 */
			void setM1M2Duty(int16_t duty1, int16_t duty2);
			/**
			 * @brief sets the M1 PID constants for velocity on Roboclaw.
			 * @param Kp   proportional
			 * @param Ki   integral
			 * @param Kd   derivative
			 * @param qpps maximum quadrature pulses per second
			 */
			void setM1VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps);
			/**
			 * @brief sets the M2 PID constants for velocity on Roboclaw.
			 * @param Kp   proportional
			 * @param Ki   integral
			 * @param Kd   derivative
			 * @param qpps Maximum quadrature pulses per second.
			 */
            void setM2VelocityPID(float Kp, float Ki, float Kd, uint32_t qpps);
			/**
			 * @brief Sets wheel speed based off provided angular velocity
			 * @param av unit of angular velocity
			 */
			void setAngularSpeed(bool motor, units::AngularVelocity av);

			/**
			 * @brief Returns the angle of the motor shaft
			 * @param  motor 0 for m1, 1 for m2
			 * @return offset angle of shaft relative to last encoder reset
			 */
			units::Angle getAngle(bool motor, bool continuous);


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
			std::shared_ptr<cmdmessenger::Command> m_read_m1_encoder_result;
			std::shared_ptr<cmdmessenger::Command> m_read_m2_encoder_result;
			std::shared_ptr<cmdmessenger::Command> m_read_m1m2_encoder_result;

			std::shared_ptr<cmdmessenger::Command> m_read_m1_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_m2_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_m1m2_encoder_speed;
			std::shared_ptr<cmdmessenger::Command> m_read_m1_encoder_speed_result;
			std::shared_ptr<cmdmessenger::Command> m_read_m2_encoder_speed_result;
			std::shared_ptr<cmdmessenger::Command> m_read_m1m2_encoder_speed_result;

			std::shared_ptr<cmdmessenger::Command> m_reset_encoders;
			std::shared_ptr<cmdmessenger::Command> m_get_buffers;
			std::shared_ptr<cmdmessenger::Command> m_get_buffers_result;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_duty;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_duty;
			std::shared_ptr<cmdmessenger::Command> m_set_m1m2_duty;

			std::shared_ptr<cmdmessenger::Command> m_set_m1_velocity_pid;
			std::shared_ptr<cmdmessenger::Command> m_set_m2_velocity_pid;

		};
	}
}

#endif // ROBOCLAW_APPENDAGE_HPP
