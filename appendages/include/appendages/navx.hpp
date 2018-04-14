#ifndef NAVX_APPENDAGE_HPP
#define NAVX_APPENDAGE_HPP

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
		class NavX : public Appendage
		{
		public:
            /**
             * @brief returns yaw
             * @return float
             */
            units::Angle getYaw();
            /**
             * @brief reports the pitch of the device
             * @return float
             */
			units::Angle getPitch();
            /**
             * @brief reports the roll of the device
             * @return float
             */
			units::Angle getRoll();
            /**
             * @brief gets the status of the device.
             * @return code representing status of device
             */
            char getStatus();
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
			NavX(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            std::shared_ptr<cmdmessenger::Command> m_get_yaw;
            std::shared_ptr<cmdmessenger::Command> m_get_yaw_result;

            std::shared_ptr<cmdmessenger::Command> m_get_pitch;
            std::shared_ptr<cmdmessenger::Command> m_get_pitch_result;

            std::shared_ptr<cmdmessenger::Command> m_get_roll;
            std::shared_ptr<cmdmessenger::Command> m_get_roll_result;

            std::shared_ptr<cmdmessenger::Command> m_get_status;
            std::shared_ptr<cmdmessenger::Command> m_get_status_result;

        };
    }
}

#endif // NAVX_APPENDAGE_HPP
