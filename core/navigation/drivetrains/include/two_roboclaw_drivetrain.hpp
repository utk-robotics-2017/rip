#ifndef TWO_ROBOCLAW_DRIVE_TRAIN_HPP
#define TWO_ROBOCLAW_DRIVE_TRAIN_HPP

#include <roboclaw.hpp>

#include "drivetrain.hpp"
#include "nav_command.hpp"

namespace rip
{
    namespace subsystem
    {
        /**
         * Abstract base class for the drive train
         */
        class TwoRoboclawDriveTrain : public Drivetrain
        {
            using Roboclaw = roboclaw::Roboclaw;
        public:
            TwoRoboclawDriveTrain(const Roboclaw& left, const Roboclaw& right)
                : m_left(left)
                , m_right(right)
            {}

            /**
             * Drive all the motors
             * @param power [-1, 1]
             */
            virtual void drive(double power) override;

            /**
             * Drive left and right separately
             * @param left [-1, 1]
             * @param right [-1, 1]
             */
            virtual void drive(double left, double right) override;

            /**
             * Drive all the motors
             *
             * all range from [-1, 1]
             */
            virtual void drive(double front_left, double front_right, double back_left, double back_rightk) override;

            /**
             * Single command to all motors
             */
            virtual void drive(const NavCommand& command) override;

            /**
             * Command left and right sides separately
             */
            virtual void drive(const NavCommand& left, const NavCommand& right) override;

            /**
             * Command four wheels separately
             */
            virtual void drive(const NavCommand& front_left, const NavCommand& front_right, const NavCommand& back_left, const NavCommand& back_right) override;

            virtual void stop() override
            {
                // todo
            }

        private:
            Roboclaw m_left;
            Roboclaw m_right;
        };
    }
}

#endif // DRIVE_TRAIN_HPP
