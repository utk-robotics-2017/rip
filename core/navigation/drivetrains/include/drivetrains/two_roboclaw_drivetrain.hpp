#ifndef TWO_ROBOCLAW_DRIVE_TRAIN_HPP
#define TWO_ROBOCLAW_DRIVE_TRAIN_HPP

#include <memory>

#include <motor_controllers/roboclaw/roboclaw.hpp>
#include <navx/navx.hpp>
#include <units/units.hpp>
#include "drivetrain.hpp"

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            /**
             * Abstract base class for the drive train
             */

            class TwoRoboclawDrivetrain : public Drivetrain
            {
                using Roboclaw = motorcontrollers::roboclaw::Roboclaw;
                using NavX = navx::NavX;
                using MotorDynamics = motorcontrollers::MotorDynamics;
            public:
                TwoRoboclawDrivetrain(const std::string& name, std::shared_ptr<Roboclaw> left,
                     std::shared_ptr<Roboclaw> right, std::shared_ptr<NavX> navx = nullptr);

                ~TwoRoboclawDrivetrain();

                /**
                 * Drive all the motors
                 * @param power [-1.0, 1.0]
                 */
                virtual void drive(double power) override;

                /**
                 * Drive left and right separately
                 * @param left [-1.0, 1.0]
                 * @param right [-1.0, 1.0]
                 */
                virtual void drive(double left, double right) override;

                /**
                 * Drive all the motors
                 *
                 * all range from [-1.0, 1.0]
                 */
                virtual void drive(double front_left, double front_right, double back_left,
                     double back_rightk) override;

                /**
                 * Single command to all motors
                 */
                virtual void drive(const MotorDynamics& command) override;

                /**
                 * Command left and right sides separately
                 */
                virtual void drive(const MotorDynamics& left, const MotorDynamics& right) override;

                /**
                 * Command four wheels separately
                 */
                virtual void drive(const MotorDynamics& front_left, const MotorDynamics& front_right,
                const MotorDynamics& back_left, const MotorDynamics& back_right) override;

                /**
                * Reads encoders for every motor you tell it to read, reports back in respective
                * order
                */
                virtual std::vector<units::Distance> readEncoders(const std::vector<Motor>& motors) override;
                /**
                * reads the encoder for one motor
                */
                virtual units::Distance readEncoder(const Motor& motor) override;
                /**
                * Reads encoder velocity for every motor you tell it to read, reports back in respective
                * order
                * @param motors list of motors to read
                */
                virtual std::vector<units::Velocity> readEncoderVelocities(const std::vector<Motor>& motors) override;
                /**
                * reads the encoder for one motor
                */
                virtual units::Velocity readEncoderVelocity(const Motor& motor) override;

                virtual units::Angle readGyro() override;

                virtual void resetEncoders() override;
                
                virtual void stop() override;

                virtual bool diagnostic() override;
            private:

                std::shared_ptr<Roboclaw> m_left;
                std::shared_ptr<Roboclaw> m_right;
                std::shared_ptr<NavX> m_navx;
            };
        }
    }
}

#endif // DRIVE_TRAIN_HPP
