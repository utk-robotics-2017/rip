#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include "nav_command.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Abstract base class for the drive train
         */
        class Drivetrain : public SubSystem
        {
        public:
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
            virtual void drive(double front_left, double front_right, double back_left, double back_rightk) = 0;

            /**
             * Single command to all motors
             */
            virtual void drive(const NavCommand& command) = 0;

            /**
             * Command left and right sides separately
             */
            virtual void drive(const NavCommand& left, const NavCommand& right) = 0;

            /**
             * Command four wheels separately
             */
            virtual void drive(const NavCommand& front_left, const NavCommand& front_right, const NavCommand& back_left, const NavCommand& back_right) = 0;

            /**
             * Stop
             *
             * @param brake whether to actively hold position or just cut power
             */
            virtual void stop(bool brake) = 0;

            virtual bool diagnostic() override;
        };
    }
}

#endif // DRIVE_TRAIN_HPP