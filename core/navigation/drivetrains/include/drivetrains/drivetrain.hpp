#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include <framework/subsystem.hpp>
#include <motor_controllers/motor_dynamics.hpp>

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            /**
             * Abstract base class for the drive train
             */
            class Drivetrain : public framework::Subsystem
            {

                using MotorDynamics = motorcontrollers::MotorDynamics;
            public:
                Drivetrain(const std::string& name)
                    : Subsystem(name)
                {}

                enum class Motor
                {
                    //4 motor drivetrain
                    kFrontLeft,
                    kFrontRight,
                    kBackLeft,
                    kBackRight,
                    //2 motor drive train
                    kLeft,
                    kRight
                }; // enum class Motor

                /**
                 * Drive all the motors
                 * @param power [-1, 1]
                 */
                virtual void drive(double power) = 0;

                /**
                 * Drive left and right separately
                 * @param left [-1, 1]
                 * @param right [-1, 1]
                 */
                virtual void drive(double left, double right) = 0;

                /**
                 * Drive all the motors
                 *
                 * all range from [-1, 1]
                 */
                virtual void drive(double front_left, double front_right, double back_left,
                    double back_right) = 0;

                /**
                 * Single command to all motors
                 */
                virtual void drive(const MotorDynamics& command) = 0;

                /**
                 * Command left and right sides separately
                 */
                virtual void drive(const MotorDynamics& left, const MotorDynamics& right) = 0;

                /**
                 * Command four wheels separately
                 */
                virtual void drive(const MotorDynamics& front_left, const MotorDynamics& front_right,
                    const MotorDynamics& back_left, const MotorDynamics& back_right) = 0;

                /**
                 * Reads encoders for every motor you tell it to read, reports back in respective
                 * order
                 */
                virtual std::vector<units::Distance> readEncoders(const std::vector<Motor>& motors) = 0;
                /**
                 * reads the encoder for one motor
                 */
                virtual units::Distance readEncoder(const Motor& motor) = 0;
                /**
                 * Reads encoder velocity for every motor you tell it to read, reports back in respective
                 * order
                 * @param motors list of motors to read
                 */
                virtual std::vector<units::Velocity> readEncoderVelocities(const std::vector<Motor>& motors) = 0;
                /**
                 * reads the encoder for one motor
                 */
                virtual units::Velocity readEncoderVelocity(const Motor& motor) = 0;
            };
        }
    }
}

#endif // DRIVE_TRAIN_HPP
