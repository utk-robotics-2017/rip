#ifndef BNO055_HPP
#define BNO055_HPP

#include <appendages/appendage.hpp>
#include <pid/pid_input.hpp>

namespace rip
{
    namespace appendages
    {
        class Bno055 : public Appendage, public pid::PidInput
        {
        public:

            units::Angle getYaw();
            units::Angle getPitch();
            units::Angle getRoll();
            bool isCalibrated();

            double get() override;

            void stop() override;

            bool diagnostic() override;

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
            Bno055(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            std::shared_ptr<cmdmessenger::Command> m_yaw;
            std::shared_ptr<cmdmessenger::Command> m_yaw_result;
            std::shared_ptr<cmdmessenger::Command> m_pitch;
            std::shared_ptr<cmdmessenger::Command> m_pitch_result;
            std::shared_ptr<cmdmessenger::Command> m_roll;
            std::shared_ptr<cmdmessenger::Command> m_roll_result;
            std::shared_ptr<cmdmessenger::Command> m_calibrated;
            std::shared_ptr<cmdmessenger::Command> m_caribrated_result;

        };
    }
}

#endif //BNO055_HPP
