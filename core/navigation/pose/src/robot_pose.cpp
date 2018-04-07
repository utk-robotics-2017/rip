#include <pose/filter_common.hpp>
#include "pose/robot_pose.hpp"
#include "geometry/point.hpp"

#include <chrono>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            RobotPose::RobotPose(const units::Distance& wheelbase, const double alpha, const double beta, const double kappa)
                    : m_odometry(new Odometry(wheelbase))
                    , m_ukf(new Ukf({alpha, kappa, beta}))
                    , m_imu(new Imu())
            {}

            void RobotPose::updateOdometry(const units::Time& time, const units::Distance& left_distance, const units::Distance& right_distance)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();

                m->updateVector() = {
                        true,  // x
                        true,  // y
                        false, // z
                        false, // roll
                        false, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        false, // vroll
                        false, // vpitch
                        true, // vyaw
                        false, // ax
                        false, // ay
                        false  // az
                };

                m_odometry->update(time, left_distance, right_distance);

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kX)) = m_odometry->x().to(units::m);
                m->measurement()(static_cast<int>(StateMembers::kY)) = m_odometry->y().to(units::m);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_odometry->theta().to(units::rad);

                m->measurement()(static_cast<int>(StateMembers::kVx)) = m_odometry->dx().to(units::m / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVy)) = m_odometry->dy().to(units::m / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVyaw)) = m_odometry->dtheta().to(units:: rad / units::s);

                // Todo: covariance

                m_ukf->processMeasurement(*(m.get()));
            }

            void RobotPose::updateOdometry(const units::Time& time, const units::Velocity& left_velocity, const units::Velocity& right_velocity)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();

                m->updateVector() = {
                        true,  // x
                        true,  // y
                        false, // z
                        false, // roll
                        false, // pitch
                        true,  // yaw
                        true, // vx
                        true, // vy
                        false, // vz
                        false, // vroll
                        false, // vpitch
                        true, // vyaw
                        false, // ax
                        false, // ay
                        false  // az
                };

                m_odometry->update(time, left_velocity, right_velocity);

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kX)) = m_odometry->x().to(units::m);
                m->measurement()(static_cast<int>(StateMembers::kY)) = m_odometry->y().to(units::m);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_odometry->theta().to(units::rad);

                m->measurement()(static_cast<int>(StateMembers::kVx)) = m_odometry->dx().to(units::m / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVy)) = m_odometry->dy().to(units::m / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVyaw)) = m_odometry->dtheta().to(units:: rad / units::s);

                // Todo: covariance


                m_ukf->processMeasurement(*(m.get()));
            }


            void RobotPose::updateImu(const units::Time& time, const units::Angle& yaw)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();
                
                m->updateVector() = {
                        false,  // x
                        false,  // y
                        false, // z
                        false, // roll
                        false, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        false, // vroll
                        false, // vpitch
                        false, // vyaw
                        false, // ax
                        false, // ay
                        false  // az
                };

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_imu->yaw().to(units::rad);

                m_ukf->processMeasurement(*(m.get()));
            }

            void RobotPose::updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();
                m->updateVector() = {
                        false,  // x
                        false,  // y
                        false, // z
                        true, // roll
                        true, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        false, // vroll
                        false, // vpitch
                        false, // vyaw
                        false, // ax
                        false, // ay
                        false  // az
                };

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_imu->yaw().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kPitch)) = m_imu->pitch().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kRoll)) = m_imu->roll().to(units::rad);

                m_ukf->processMeasurement(*(m.get()));
            }

            void RobotPose::updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::AngularVelocity& yaw_velocity, const units::AngularVelocity& pitch_velocity, const units::AngularVelocity& roll_velocity)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();
                m->updateVector() = {
                        false,  // x
                        false,  // y
                        false, // z
                        true, // roll
                        true, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        true, // vroll
                        true, // vpitch
                        true, // vyaw
                        false, // ax
                        false, // ay
                        false  // az
                };

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_imu->yaw().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kPitch)) = m_imu->pitch().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kRoll)) = m_imu->roll().to(units::rad);

                m->measurement()(static_cast<int>(StateMembers::kVyaw)) = m_imu->dYaw().to(units::rad / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVpitch)) = m_imu->dPitch().to(units::rad / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVroll)) = m_imu->dRoll().to(units::rad / units::s);


                m_ukf->processMeasurement(*(m.get()));
            }

            void RobotPose::updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::Acceleration& x, const units::Acceleration& y, const units::Acceleration& z)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();
                m->updateVector() = {
                        false,  // x
                        false,  // y
                        false, // z
                        true, // roll
                        true, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        false, // vroll
                        false, // vpitch
                        false, // vyaw
                        true, // ax
                        true, // ay
                        true  // az
                };

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_imu->yaw().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kPitch)) = m_imu->pitch().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kRoll)) = m_imu->roll().to(units::rad);

                m->measurement()(static_cast<int>(StateMembers::kAx)) = m_imu->ddX().to(units::m / units::s / units::s);
                m->measurement()(static_cast<int>(StateMembers::kAy)) = m_imu->ddY().to(units::m / units::s / units::s);
                m->measurement()(static_cast<int>(StateMembers::kAz)) = m_imu->ddZ().to(units::m / units::s / units::s);

                m_ukf->processMeasurement(*(m.get()));
            }

            void RobotPose::updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::AngularVelocity& yaw_velocity, const units::AngularVelocity& pitch_velocity, const units::AngularVelocity& roll_velocity, const units::Acceleration& x, const units::Acceleration& y, const units::Acceleration& z)
            {
                std::shared_ptr<Measurement> m = std::make_shared<Measurement>();
                m->updateVector() = {
                        false,  // x
                        false,  // y
                        false, // z
                        true, // roll
                        true, // pitch
                        true,  // yaw
                        false, // vx
                        false, // vy
                        false, // vz
                        true, // vroll
                        true, // vpitch
                        true, // vyaw
                        true, // ax
                        true, // ay
                        true  // az
                };

                Eigen::VectorXd measurement(kStateSize);
                Eigen::MatrixXd measurementCovariance(kStateSize, kStateSize);
                measurement.setZero();
                measurementCovariance.setZero();

                m->time() = time.to(units::s);
                m->measurement()(static_cast<int>(StateMembers::kYaw)) = m_imu->yaw().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kPitch)) = m_imu->pitch().to(units::rad);
                m->measurement()(static_cast<int>(StateMembers::kRoll)) = m_imu->roll().to(units::rad);

                m->measurement()(static_cast<int>(StateMembers::kVyaw)) = m_imu->dYaw().to(units::rad / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVpitch)) = m_imu->dPitch().to(units::rad / units::s);
                m->measurement()(static_cast<int>(StateMembers::kVroll)) = m_imu->dRoll().to(units::rad / units::s);

                m->measurement()(static_cast<int>(StateMembers::kAx)) = m_imu->ddX().to(units::m / units::s / units::s);
                m->measurement()(static_cast<int>(StateMembers::kAy)) = m_imu->ddY().to(units::m / units::s / units::s);
                m->measurement()(static_cast<int>(StateMembers::kAz)) = m_imu->ddZ().to(units::m / units::s / units::s);

                m_ukf->processMeasurement(*(m.get()));
            }

            units::Distance RobotPose::x()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kX)) * units::m;
            }

            units::Distance RobotPose::y()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kY)) * units::m;
            }

            units::Distance RobotPose::z()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kZ)) * units::m;
            }

            units::Angle RobotPose::yaw()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kYaw)) * units::rad;
            }

            units::Angle RobotPose::pitch()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kPitch)) * units::rad;
            }

            units::Angle RobotPose::roll()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kRoll)) * units::rad;
            }

            units::Velocity RobotPose::dx()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVx)) * units::m / units::s;
            }

            units::Velocity RobotPose::dy()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVy)) * units::m / units::s;
            }

            units::Velocity RobotPose::dz()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVz)) * units::m / units::s;
            }

            units::AngularVelocity RobotPose::dYaw()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVyaw)) * units::rad / units::s;
            }

            units::AngularVelocity RobotPose::dPitch()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVpitch)) * units::rad / units::s;
            }

            units::AngularVelocity RobotPose::dRoll()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kVroll)) * units::rad / units::s;
            }

            units::Acceleration RobotPose::ddx()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kAx)) * units::m / units::s / units::s;
            }

            units::Acceleration RobotPose::ddy()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kAy)) * units::m / units::s / units::s;
            }

            units::Acceleration RobotPose::ddz()
            {
                updatePredictions();
                return m_ukf->getState()(static_cast<int>(StateMembers::kAz)) * units::m / units::s / units::s;
            }

            Pose RobotPose::pose()
            {
                updatePredictions();
                const Eigen::VectorXd& state = m_ukf->getState();

                Pose pose;

                pose.x = state(static_cast<int>(StateMembers::kX)) * units::m;
                pose.y = state(static_cast<int>(StateMembers::kY)) * units::m;
                pose.z = state(static_cast<int>(StateMembers::kZ)) * units::m;

                pose.yaw = state(static_cast<int>(StateMembers::kYaw)) * units::rad;
                pose.pitch = state(static_cast<int>(StateMembers::kPitch)) * units::rad;
                pose.roll = state(static_cast<int>(StateMembers::kRoll)) * units::rad;

                pose.d_x = state(static_cast<int>(StateMembers::kVx)) * units::m / units::s;
                pose.d_y = state(static_cast<int>(StateMembers::kVy)) * units::m / units::s;
                pose.d_z = state(static_cast<int>(StateMembers::kVz)) * units::m / units::s;

                pose.d_yaw = state(static_cast<int>(StateMembers::kVyaw)) * units::rad / units::s;
                pose.d_pitch = state(static_cast<int>(StateMembers::kVpitch)) * units::rad / units::s;
                pose.d_roll = state(static_cast<int>(StateMembers::kVroll)) * units::rad / units::s;

                pose.dd_x = state(static_cast<int>(StateMembers::kAx)) * units::m / units::s / units::s;
                pose.dd_y = state(static_cast<int>(StateMembers::kAy)) * units::m / units::s / units::s;
                pose.dd_z = state(static_cast<int>(StateMembers::kAz)) * units::m / units::s / units::s;

                return pose;
            }

            void RobotPose::updatePredictions()
            {
                const units::Time now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                const units::Time dt = m_last_update;
                m_ukf->predict(now.to(units::s), dt.to(units::s));
                m_last_update = now;
            }
        }
    }
}