#ifndef TWO_ROBOCLAW_DRIVE_TRAIN_HPP
#define TWO_ROBOCLAW_DRIVE_TRAIN_HPP

#include <memory>
#include <appendages/roboclaw.hpp>
#include <navx/navx.hpp>
#include <units/units.hpp>
#include "drivetrain.hpp"
#include <motor_controllers/motor_dynamics.hpp>
#include <tuple>


namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            /**
             * Abstract base class for the drive train
             */

            class TwoRoboclawAppendageDrivetrain : public Drivetrain
            {
                using Roboclaw = rip::appendages::Roboclaw;
                using NavX = navx::NavX;
                using MotorDynamics = motorcontrollers::MotorDynamics;
            public:
                TwoRoboclawAppendageDrivetrain(const std::string& name, std::shared_ptr<Roboclaw> left,
                                               std::shared_ptr<Roboclaw> right, double ticks_per_rev, units::Distance wheel_radius,
                                               std::shared_ptr<NavX> navx = nullptr);

                ~TwoRoboclawAppendageDrivetrain();

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
                /**
                 * @brief Given a MotorDynamics object, does PID driving with units for one motor.
                 * @param dynamics      See MotorDynamics
                 * @param respectBuffer if false, command will be buffered
                 */
                void setDynamics(const Motor& motor, const MotorDynamics& dynamics, bool respectBuffer = 1);
                /**
                 * @brief Given a MotorDynamics object, does PID driving with units for one motor.
                 * @param dynamics      See MotorDynamics
                 * @param respectBuffer if false, command will be buffered
                 */
                void setDynamics(const MotorDynamics& dynamics, bool respectBuffer = 1); //Not implemented on arduino
                /**
                 * @brief returns the encoder distance and velocity in a pair
                 * @param motor 0 for motor 1, 1 for motor 2
                 */
                std::tuple<units::Distance, units::Velocity> getDistAndVel(const Motor& motor);
                /**
                 * [getDistAndVel description]
                 * @param side [description]
                 */
                std::tuple<units::Distance, units::Velocity> getDistAndVel(bool side);
                /**
                 * Resets the encoders
                 */
                virtual void resetEncoders() override;

                virtual units::Angle readGyro() const;

                virtual void stop() override;

                virtual bool diagnostic() override;
            private:

                std::shared_ptr<Roboclaw> m_left;
                std::shared_ptr<Roboclaw> m_right;
                std::shared_ptr<NavX> m_navx;
                double m_ticks_per_rev;
                units::Distance m_wheel_radius;
            };
        }
    }
}

#endif // DRIVE_TRAIN_HPP
