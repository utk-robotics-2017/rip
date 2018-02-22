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
                    double back_rightk) = 0;

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
            };
        }
    }
}

#endif // DRIVE_TRAIN_HPP
