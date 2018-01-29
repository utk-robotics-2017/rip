#include <teb_planner_gui/config_widget.hpp>
#include "ui_config_widget.h"

#include <QInputDialog>


#include <fmt/format.h>

#include <teb_planner_gui/exceptions.hpp>

#include <teb_planner_gui/exceptions.hpp>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
#define setupMap(map, var_name) \
                map[m_ui->var_name] = #var_name;

            ConfigWidget::ConfigWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::ConfigWidget)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);

                // Disable

                // Trajectory
                m_ui->autosize->setEnabled(false);
                m_ui->dt_ref->setEnabled(false);
                m_ui->dt_hysteresis->setEnabled(false);
                m_ui->min_samples->setEnabled(false);
                m_ui->max_samples->setEnabled(false);
                m_ui->overwrite_global_plan->setEnabled(false);
                m_ui->backwards_motion->setEnabled(false);
                m_ui->global_plan_waypoint_separation->setEnabled(false);
                m_ui->waypoints_ordered->setEnabled(false);
                m_ui->max_global_plan_lookahead_distance->setEnabled(false);
                m_ui->exact_arc_length->setEnabled(false);
                m_ui->force_new_goal_distance->setEnabled(false);
                m_ui->feasibility_check_num_poses->setEnabled(false);

                // Robot
                m_ui->max_velocity_x->setEnabled(false);
                m_ui->max_velocity_x_backwards->setEnabled(false);
                m_ui->max_velocity_y->setEnabled(false);
                m_ui->max_velocity_theta->setEnabled(false);
                m_ui->acceleration_limit_x->setEnabled(false);
                m_ui->acceleration_limit_y->setEnabled(false);
                m_ui->acceleration_limit_theta->setEnabled(false);
                m_ui->min_turning_radius->setEnabled(false);
                m_ui->wheelbase->setEnabled(false);

                // Goal Tolerance
                m_ui->yaw_goal_tolerance->setEnabled(false);
                m_ui->xy_goal_tolerance->setEnabled(false);
                m_ui->free_goal_velocity->setEnabled(false);

                // Obstacles
                m_ui->min_obstacle_distance->setEnabled(false);
                m_ui->inflation_distance->setEnabled(false);
                m_ui->dynamic_obstacle_inflation_distance->setEnabled(false);
                m_ui->include_dynamic_obstacles->setEnabled(false);
                m_ui->obstacles_poses_affected->setEnabled(false);
                m_ui->obstacle_association_force_inclusion_factor->setEnabled(false);
                m_ui->obstacle_association_cutoff_factor->setEnabled(false);

                // Optimization
                m_ui->num_inner_iterations->setEnabled(false);
                m_ui->num_outer_iterations->setEnabled(false);
                m_ui->penalty_epsilon->setEnabled(false);
                m_ui->max_velocity_x_weight->setEnabled(false);
                m_ui->max_velocity_y_weight->setEnabled(false);
                m_ui->max_velocity_theta_weight->setEnabled(false);
                m_ui->acceleration_limit_x_weight->setEnabled(false);
                m_ui->acceleration_limit_y_weight->setEnabled(false);
                m_ui->acceleration_limit_theta_weight->setEnabled(false);
                m_ui->kinematics_nh_weight->setEnabled(false);
                m_ui->kinematics_forward_drive_weight->setEnabled(false);
                m_ui->kinematics_turning_radius_weight->setEnabled(false);
                m_ui->optimal_time_weight->setEnabled(false);
                m_ui->obstacle_weight->setEnabled(false);
                m_ui->inflation_weight->setEnabled(false);
                m_ui->dynamic_obstacle_weight->setEnabled(false);
                m_ui->dynamic_obstacle_inflation_weight->setEnabled(false);
                m_ui->waypoint_weight->setEnabled(false);
                m_ui->weight_adapt_factor->setEnabled(false);


                // Trajectory
                setupMap(m_doubles, autosize);
                setupMap(m_times, dt_ref);
                setupMap(m_times, dt_hysteresis);
                setupMap(m_integers, min_samples);
                setupMap(m_integers, max_samples);
                m_bools[m_ui->overwrite_global_plan] = "global_plan_overwrite_orientation";
                setupMap(m_distances, global_plan_waypoint_separation);
                setupMap(m_bools, waypoints_ordered);
                setupMap(m_distances, max_global_plan_lookahead_distance);
                setupMap(m_bools, exact_arc_length);
                m_distances[m_ui->force_new_goal_distance] = "force_reinit_new_goal_distance";
                setupMap(m_integers, feasibility_check_num_poses);

                // Robot
                setupMap(m_velocities, max_velocity_x);
                setupMap(m_velocities, max_velocity_x_backwards);
                setupMap(m_velocities, max_velocity_y);
                setupMap(m_angular_velocities, max_velocity_theta);
                setupMap(m_accelerations, acceleration_limit_x);
                setupMap(m_accelerations, acceleration_limit_y);
                setupMap(m_angular_accelerations, acceleration_limit_theta);
                setupMap(m_distances, min_turning_radius);
                setupMap(m_distances, wheelbase);

                // Goal Tolerance
                setupMap(m_angles, yaw_goal_tolerance);
                setupMap(m_distances, xy_goal_tolerance);
                setupMap(m_bools, free_goal_velocity);

                // Obstacles
                setupMap(m_distances, min_obstacle_distance);
                setupMap(m_distances, inflation_distance);
                setupMap(m_distances, dynamic_obstacle_inflation_distance);
                setupMap(m_bools, include_dynamic_obstacles);
                setupMap(m_integers, obstacles_poses_affected);
                setupMap(m_doubles, obstacle_association_force_inclusion_factor);
                setupMap(m_doubles, obstacle_association_cutoff_factor);

                // Optimization
                setupMap(m_integers, num_inner_iterations);
                setupMap(m_integers, num_outer_iterations);
                setupMap(m_doubles, penalty_epsilon);
                setupMap(m_doubles, max_velocity_x_weight);
                setupMap(m_doubles, max_velocity_y_weight);
                setupMap(m_doubles, max_velocity_theta_weight);
                setupMap(m_doubles, acceleration_limit_x_weight);
                setupMap(m_doubles, acceleration_limit_y_weight);
                setupMap(m_doubles, acceleration_limit_theta_weight);
                setupMap(m_doubles, kinematics_nh_weight);
                setupMap(m_doubles, kinematics_forward_drive_weight);
                setupMap(m_doubles, kinematics_turning_radius_weight);
                setupMap(m_doubles, optimal_time_weight);
                setupMap(m_doubles, obstacle_weight);
                setupMap(m_doubles, inflation_weight);
                setupMap(m_doubles, dynamic_obstacle_weight);
                setupMap(m_doubles, dynamic_obstacle_inflation_weight);
                setupMap(m_doubles, waypoint_weight);
                setupMap(m_doubles, weight_adapt_factor);

                for (auto iter : m_distances)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateDistance()));
                }

                for (auto iter : m_times)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateTime()));
                }

                for (auto iter : m_velocities)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateVelocity()));
                }

                for (auto iter : m_accelerations)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateAcceleration()));
                }

                for (auto iter : m_doubles)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateDouble()));
                }

                for (auto iter : m_integers)
                {
                    connect(iter.first, SIGNAL(textEdited(QString)), this, SLOT(updateInteger()));
                }

                for (auto iter : m_bools)
                {
                    connect(iter.first, SIGNAL(stateChanged(int)), this, SLOT(updateBool()));
                }

                connect(m_ui->add_config, SIGNAL(clicked(bool)), this, SLOT(addConfig()));
                connect(m_ui->config_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setConfig(QString)));
                connect(m_ui->delete_config, SIGNAL(clicked(bool)), this, SLOT(removeConfig()));
            }

            void ConfigWidget::setOptions()
            {
                QStringList names;
                for(const std::string& name : m_settings->getConfigNames())
                {
                    names << QString(name.c_str());
                }
                m_ui->config_options->addItems(names);
            }

            void ConfigWidget::updateDistance()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::Distance dist = value * units::in;

                    std::string var_name = m_distances[line_edit];
                    if (var_name == "global_plan_waypoint_separation")
                    {
                        m_current->trajectory.global_plan_waypoint_separation = dist;
                    }
                    else if (var_name == "max_global_plan_lookahead_distance")
                    {
                        m_current->trajectory.max_global_plan_lookahead_distance = dist;
                    }
                    else if (var_name == "force_reinit_new_goal_distance")
                    {
                        m_current->trajectory.force_reinit_new_goal_distance = dist;
                    }
                    else if (var_name == "min_turning_radius")
                    {
                        m_current->robot.min_turning_radius = dist;
                    }
                    else if (var_name == "wheelbase")
                    {
                        m_current->robot.wheelbase = dist;
                    }
                    else if (var_name == "xy_goal_tolerance")
                    {
                        m_current->goal_tolerance.xy_goal_tolerance;
                    }
                    else if (var_name == "min_obstacle_distance")
                    {
                        m_current->obstacles.min_obstacle_distance = dist;
                    }
                    else if (var_name == "inflation_distance")
                    {
                        m_current->obstacles.inflation_distance = dist;
                    }
                    else if (var_name == "dynamic_obstacle_inflation_distance")
                    {
                        m_current->obstacles.dynamic_obstacle_inflation_distance = dist;
                    }
                    else
                    {
                        throw UnknownDistanceSettingException(fmt::format("Unknown distance setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateTime()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::Time time = value * units::s;

                    std::string var_name = m_times[line_edit];
                    if (var_name == "dt_ref")
                    {
                        m_current->trajectory.dt_ref = time;
                    }
                    else if (var_name == "dt_hysteresis")
                    {
                        m_current->trajectory.dt_hysteresis = time;
                    }
                    else
                    {
                        throw UnknownTimeSettingException(fmt::format("Unknown time setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateVelocity()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::Velocity vel = value * units::in / units::s;

                    std::string var_name = m_velocities[line_edit];
                    if (var_name == "max_velocity_x")
                    {
                        m_current->robot.max_velocity_x = vel;
                    }
                    else if (var_name == "max_velocity_x_backwards")
                    {
                        m_current->robot.max_velocity_x_backwards = vel;
                    }
                    else if (var_name == "max_velocity_y")
                    {
                        m_current->robot.max_velocity_y = vel;
                    }
                    else
                    {
                        throw UnknownVelocitySettingException(fmt::format("Unknown velocity setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateAcceleration()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::Acceleration acc = value * units::in / units::s / units::s;
                    std::string var_name = m_accelerations[line_edit];
                    if (var_name == "acceleration_limit_x")
                    {
                        m_current->robot.acceleration_limit_x = acc;
                    }
                    else if (var_name == "acceleration_limit_y")
                    {
                        m_current->robot.acceleration_limit_y = acc;
                    }
                    else
                    {
                        throw UnknownAccelerationSettingException(fmt::format("Unknown acceleration setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateAngle()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::Angle angle = value * units::deg;
                    std::string var_name = m_angles[line_edit];
                    if (var_name == "yaw_goal_tolerance")
                    {
                        m_current->goal_tolerance.yaw_goal_tolerance = angle;
                    }
                    else
                    {
                        throw UnknownAngleSettingException(fmt::format("Unknown angle setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateAngularVelocity()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::AngularVelocity vel = value * units::deg / units::s;
                    std::string var_name = m_angular_velocities[line_edit];
                    if (var_name == "max_velocity_theta")
                    {
                        m_current->robot.max_velocity_theta = vel;
                    }
                    else
                    {
                        throw UnknownAngularVelocitySettingException(fmt::format("Unknown angular velocity setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateAngularAcceleration()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    units::AngularAcceleration acc = value * units::deg / units::s / units::s;
                    std::string var_name = m_angular_accelerations[line_edit];
                    if (var_name == "acceleration_limits_theta")
                    {
                        m_current->robot.acceleration_limit_theta = acc;
                    }
                    else
                    {
                        throw UnknownAngularAccelerationSettingException(fmt::format("Unknown angular acceleration setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateDouble()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                double value = text.toDouble(&ok);
                if (ok)
                {
                    std::string var_name = m_doubles[line_edit];
                    if (var_name == "autosize")
                    {
                        m_current->trajectory.autosize = value;
                    }
                    else if (var_name == "obstacle_association_force_inclusion_factor")
                    {
                        m_current->obstacles.obstacle_association_force_inclusion_factor = value;
                    }
                    else if (var_name == "obstacle_association_cutoff_factor")
                    {
                        m_current->obstacles.obstacle_association_cutoff_factor = value;
                    }
                    else if (var_name == "penalty_epsilon")
                    {
                        m_current->optimization.penalty_epsilon = value;
                    }
                    else if (var_name == "max_velocity_x_weight")
                    {
                        m_current->optimization.max_velocity_x_weight = value;
                    }
                    else if (var_name == "max_velocity_y_weight")
                    {
                        m_current->optimization.max_velocity_y_weight = value;
                    }
                    else if (var_name == "max_velocity_theta_weight")
                    {
                        m_current->optimization.max_velocity_theta_weight = value;
                    }
                    else if (var_name == "acceleration_limit_x_weight")
                    {
                        m_current->optimization.acceleration_limit_x_weight = value;
                    }
                    else if (var_name == "acceleration_limit_y_weight")
                    {
                        m_current->optimization.acceleration_limit_y_weight = value;
                    }
                    else if (var_name == "acceleration_limit_theta_weight")
                    {
                        m_current->optimization.acceleration_limit_theta_weight = value;
                    }
                    else if (var_name == "kinematics_nh_weight")
                    {
                        m_current->optimization.kinematics_nh_weight = value;
                    }
                    else if (var_name == "kinematics_forward_drive_weight")
                    {
                        m_current->optimization.kinematics_forward_drive_weight = value;
                    }
                    else if (var_name == "kinematics_turning_radius_weight")
                    {
                        m_current->optimization.kinematics_turning_radius_weight = value;
                    }
                    else if (var_name == "optimal_time_weight")
                    {
                        m_current->optimization.optimal_time_weight = value;
                    }
                    else if (var_name == "obstacle_weight")
                    {
                        m_current->optimization.obstacle_weight = value;
                    }
                    else if (var_name == "inflation_weight")
                    {
                        m_current->optimization.inflation_weight = value;
                    }
                    else if (var_name == "dynamic_obstacle_weight")
                    {
                        m_current->optimization.dynamic_obstacle_weight = value;
                    }
                    else if (var_name == "dynamic_obstacle_inflation_weight")
                    {
                        m_current->optimization.dynamic_obstacle_inflation_weight = value;
                    }
                    else if (var_name == "waypoint_weight")
                    {
                        m_current->optimization.waypoint_weight = value;
                    }
                    else if (var_name == "weight_adapt_factor")
                    {
                        m_current->optimization.weight_adapt_factor = value;
                    }
                    else
                    {
                        throw UnknownDoubleSettingException(fmt::format("Unknown double setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateInteger()
            {
                if (!m_current)
                {
                    return;
                }

                QLineEdit* line_edit = static_cast<QLineEdit*>(sender());
                QString text = line_edit->text();
                bool ok;
                int value = text.toInt(&ok);
                if (ok)
                {
                    std::string var_name = m_integers[line_edit];
                    if (var_name == "min_samples")
                    {
                        m_current->trajectory.min_samples = value;
                    }
                    else if (var_name == "max_samples")
                    {
                        m_current->trajectory.max_samples = value;
                    }
                    else if (var_name == "feasibility_check_num_poses")
                    {
                        m_current->trajectory.feasibility_check_no_poses = value;
                    }
                    else if (var_name == "obstacles_poses_affected")
                    {
                        m_current->obstacles.obstacles_poses_affected = value;
                    }
                    else if (var_name == "num_inner_iterations")
                    {
                        m_current->optimization.num_inner_iterations = value;
                    }
                    else if (var_name == "num_outer_iterations")
                    {
                        m_current->optimization.num_outer_iterations = value;
                    }
                    else
                    {
                        throw UnknownIntegerSettingException(fmt::format("Unknown integer setting: {}", var_name));
                    }
                }
            }

            void ConfigWidget::updateBool()
            {
                if (!m_current)
                {
                    return;
                }

                QCheckBox* check_box = static_cast<QCheckBox*>(sender());
                bool value = check_box->isChecked();
                std::string var_name = m_bools[check_box];
                if (var_name == "global_plan_overwrite_orientation")
                {
                    m_current->trajectory.global_plan_overwrite_orientation = value;
                }
                else if (var_name == "allow_init_with_backwards_motion")
                {
                    m_current->trajectory.allow_init_with_backwards_motion = value;
                }
                else if (var_name == "waypoints_ordered")
                {
                    m_current->trajectory.waypoints_ordered = value;
                }
                else if (var_name == "exact_arc_length")
                {
                    m_current->trajectory.exact_arc_length = value;
                }
                else if (var_name == "free_goal_velocity")
                {
                    m_current->goal_tolerance.free_goal_velocity = value;
                }
                else if (var_name == "include_dynamic_obstacles")
                {
                    m_current->obstacles.include_dynamic_obstacles = value;
                }
                else
                {
                    throw UnknownBooleanSettingException(fmt::format("Unknown boolean setting: {}", var_name));
                }
            }

            void ConfigWidget::addConfig()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Config"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if (ok && !name.isEmpty())
                {
                    m_settings->addConfig(name.toStdString());
                    m_ui->config_options->addItem(name);
                    m_name = name.toStdString();
                }
            }

            void ConfigWidget::setConfig(const QString& name)
            {
                m_name = name.toStdString();
                m_current = m_settings->config(m_name);
                if (m_current)
                {
                    // Enable

                    // Trajectory
                    m_ui->autosize->setEnabled(true);
                    m_ui->dt_ref->setEnabled(true);
                    m_ui->dt_hysteresis->setEnabled(true);
                    m_ui->min_samples->setEnabled(true);
                    m_ui->max_samples->setEnabled(true);
                    m_ui->overwrite_global_plan->setEnabled(true);
                    m_ui->backwards_motion->setEnabled(true);
                    m_ui->global_plan_waypoint_separation->setEnabled(true);
                    m_ui->waypoints_ordered->setEnabled(true);
                    m_ui->max_global_plan_lookahead_distance->setEnabled(true);
                    m_ui->exact_arc_length->setEnabled(true);
                    m_ui->force_new_goal_distance->setEnabled(true);
                    m_ui->feasibility_check_num_poses->setEnabled(true);

                    // Robot
                    m_ui->max_velocity_x->setEnabled(true);
                    m_ui->max_velocity_x_backwards->setEnabled(true);
                    m_ui->max_velocity_y->setEnabled(true);
                    m_ui->max_velocity_theta->setEnabled(true);
                    m_ui->acceleration_limit_x->setEnabled(true);
                    m_ui->acceleration_limit_y->setEnabled(true);
                    m_ui->acceleration_limit_theta->setEnabled(true);
                    m_ui->min_turning_radius->setEnabled(true);
                    m_ui->wheelbase->setEnabled(true);

                    // Goal Tolerance
                    m_ui->yaw_goal_tolerance->setEnabled(true);
                    m_ui->xy_goal_tolerance->setEnabled(true);
                    m_ui->free_goal_velocity->setEnabled(true);

                    // Obstacles
                    m_ui->min_obstacle_distance->setEnabled(true);
                    m_ui->inflation_distance->setEnabled(true);
                    m_ui->dynamic_obstacle_inflation_distance->setEnabled(true);
                    m_ui->include_dynamic_obstacles->setEnabled(true);
                    m_ui->obstacles_poses_affected->setEnabled(true);
                    m_ui->obstacle_association_force_inclusion_factor->setEnabled(true);
                    m_ui->obstacle_association_cutoff_factor->setEnabled(true);

                    // Optimization
                    m_ui->num_inner_iterations->setEnabled(true);
                    m_ui->num_outer_iterations->setEnabled(true);
                    m_ui->penalty_epsilon->setEnabled(true);
                    m_ui->max_velocity_x_weight->setEnabled(true);
                    m_ui->max_velocity_y_weight->setEnabled(true);
                    m_ui->max_velocity_theta_weight->setEnabled(true);
                    m_ui->acceleration_limit_x_weight->setEnabled(true);
                    m_ui->acceleration_limit_y_weight->setEnabled(true);
                    m_ui->acceleration_limit_theta_weight->setEnabled(true);
                    m_ui->kinematics_nh_weight->setEnabled(true);
                    m_ui->kinematics_forward_drive_weight->setEnabled(true);
                    m_ui->kinematics_turning_radius_weight->setEnabled(true);
                    m_ui->optimal_time_weight->setEnabled(true);
                    m_ui->obstacle_weight->setEnabled(true);
                    m_ui->inflation_weight->setEnabled(true);
                    m_ui->dynamic_obstacle_weight->setEnabled(true);
                    m_ui->dynamic_obstacle_inflation_weight->setEnabled(true);
                    m_ui->waypoint_weight->setEnabled(true);
                    m_ui->weight_adapt_factor->setEnabled(true);

                    // Set

                    // Trajectory
                    m_ui->autosize->setText(QString::number(m_current->trajectory.autosize, 'f', 2));
                    m_ui->dt_ref->setText(QString::number(m_current->trajectory.dt_ref.to(units::s), 'f', 2));
                    m_ui->dt_hysteresis->setText(QString::number(m_current->trajectory.dt_hysteresis.to(units::s), 'f', 2));
                    m_ui->min_samples->setText(QString::number(m_current->trajectory.min_samples));
                    m_ui->max_samples->setText(QString::number(m_current->trajectory.max_samples));
                    m_ui->overwrite_global_plan->setChecked(m_current->trajectory.global_plan_overwrite_orientation);
                    m_ui->backwards_motion->setChecked(m_current->trajectory.allow_init_with_backwards_motion);
                    m_ui->global_plan_waypoint_separation->setText(QString::number(m_current->trajectory.global_plan_waypoint_separation.to(units::in), 'f', 2));
                    m_ui->waypoints_ordered->setChecked(m_current->trajectory.waypoints_ordered);
                    m_ui->max_global_plan_lookahead_distance->setText(QString::number(m_current->trajectory.max_global_plan_lookahead_distance.to(units::in), 'f', 2));
                    m_ui->exact_arc_length->setChecked(m_current->trajectory.exact_arc_length);
                    m_ui->force_new_goal_distance->setText(QString::number(m_current->trajectory.force_reinit_new_goal_distance.to(units::in), 'f', 2));
                    m_ui->feasibility_check_num_poses->setText(QString::number(m_current->trajectory.feasibility_check_no_poses));

                    // Robot
                    m_ui->max_velocity_x->setText(QString::number(m_current->robot.max_velocity_x.to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_x_backwards->setText(QString::number(m_current->robot.max_velocity_x_backwards.to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_y->setText(QString::number(m_current->robot.max_velocity_y.to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_theta->setText(QString::number(m_current->robot.max_velocity_theta.to(units::deg / units::s), 'f', 2));
                    m_ui->acceleration_limit_x->setText(QString::number(m_current->robot.acceleration_limit_x.to(units::in / units::s / units::s), 'f', 2));
                    m_ui->acceleration_limit_y->setText(QString::number(m_current->robot.acceleration_limit_y.to(units::in / units::s / units::s), 'f', 2));
                    m_ui->acceleration_limit_theta->setText(QString::number(m_current->robot.acceleration_limit_theta.to(units::deg / units::s / units::s), 'f', 2));
                    m_ui->min_turning_radius->setText(QString::number(m_current->robot.min_turning_radius.to(units::in), 'f', 2));
                    m_ui->wheelbase->setText(QString::number(m_current->robot.wheelbase.to(units::in), 'f', 2));

                    // Goal Tolerance
                    m_ui->yaw_goal_tolerance->setText(QString::number(m_current->goal_tolerance.yaw_goal_tolerance.to(units::deg), 'f', 2));
                    m_ui->xy_goal_tolerance->setText(QString::number(m_current->goal_tolerance.xy_goal_tolerance.to(units::in), 'f', 2));
                    m_ui->free_goal_velocity->setChecked(m_current->goal_tolerance.free_goal_velocity);

                    // Obstacles
                    m_ui->min_obstacle_distance->setText(QString::number(m_current->obstacles.min_obstacle_distance.to(units::in), 'f', 2));
                    m_ui->inflation_distance->setText(QString::number(m_current->obstacles.inflation_distance.to(units::in), 'f', 2));
                    m_ui->dynamic_obstacle_inflation_distance->setText(QString::number(m_current->obstacles.dynamic_obstacle_inflation_distance.to(units::in), 'f', 2));
                    m_ui->include_dynamic_obstacles->setChecked(m_current->obstacles.include_dynamic_obstacles);
                    m_ui->obstacles_poses_affected->setText(QString::number(m_current->obstacles.obstacles_poses_affected));
                    m_ui->obstacle_association_force_inclusion_factor->setText(QString::number(m_current->obstacles.obstacle_association_force_inclusion_factor, 'f', 2));
                    m_ui->obstacle_association_cutoff_factor->setText(QString::number(m_current->obstacles.obstacle_association_cutoff_factor, 'f', 2));

                    // Optimization
                    m_ui->num_inner_iterations->setText(QString::number(m_current->optimization.num_inner_iterations));
                    m_ui->num_outer_iterations->setText(QString::number(m_current->optimization.num_outer_iterations));
                    m_ui->penalty_epsilon->setText(QString::number(m_current->optimization.penalty_epsilon, 'f', 2));
                    m_ui->max_velocity_x_weight->setText(QString::number(m_current->optimization.max_velocity_x_weight, 'f', 2));
                    m_ui->max_velocity_y_weight->setText(QString::number(m_current->optimization.max_velocity_y_weight, 'f', 2));
                    m_ui->max_velocity_theta_weight->setText(QString::number(m_current->optimization.max_velocity_theta_weight, 'f', 2));
                    m_ui->acceleration_limit_x_weight->setText(QString::number(m_current->optimization.acceleration_limit_x_weight, 'f', 2));
                    m_ui->acceleration_limit_y_weight->setText(QString::number(m_current->optimization.acceleration_limit_y_weight, 'f', 2));
                    m_ui->acceleration_limit_theta_weight->setText(QString::number(m_current->optimization.acceleration_limit_theta_weight, 'f', 2));
                    m_ui->kinematics_nh_weight->setText(QString::number(m_current->optimization.kinematics_nh_weight, 'f', 2));
                    m_ui->kinematics_forward_drive_weight->setText(QString::number(m_current->optimization.kinematics_forward_drive_weight, 'f', 2));
                    m_ui->kinematics_turning_radius_weight->setText(QString::number(m_current->optimization.kinematics_turning_radius_weight, 'f', 2));
                    m_ui->optimal_time_weight->setText(QString::number(m_current->optimization.optimal_time_weight, 'f', 2));
                    m_ui->obstacle_weight->setText(QString::number(m_current->optimization.obstacle_weight, 'f', 2));
                    m_ui->inflation_weight->setText(QString::number(m_current->optimization.inflation_weight, 'f', 2));
                    m_ui->dynamic_obstacle_weight->setText(QString::number(m_current->optimization.dynamic_obstacle_weight, 'f', 2));
                    m_ui->dynamic_obstacle_inflation_weight->setText(QString::number(m_current->optimization.dynamic_obstacle_inflation_weight, 'f', 2));
                    m_ui->waypoint_weight->setText(QString::number(m_current->optimization.waypoint_weight, 'f', 2));
                    m_ui->weight_adapt_factor->setText(QString::number(m_current->optimization.weight_adapt_factor, 'f', 2));
                }
                else
                {
                    // Disable

                    // Trajectory
                    m_ui->autosize->setEnabled(false);
                    m_ui->dt_ref->setEnabled(false);
                    m_ui->dt_hysteresis->setEnabled(false);
                    m_ui->min_samples->setEnabled(false);
                    m_ui->max_samples->setEnabled(false);
                    m_ui->overwrite_global_plan->setEnabled(false);
                    m_ui->backwards_motion->setEnabled(false);
                    m_ui->global_plan_waypoint_separation->setEnabled(false);
                    m_ui->waypoints_ordered->setEnabled(false);
                    m_ui->max_global_plan_lookahead_distance->setEnabled(false);
                    m_ui->exact_arc_length->setEnabled(false);
                    m_ui->force_new_goal_distance->setEnabled(false);
                    m_ui->feasibility_check_num_poses->setEnabled(false);

                    // Robot
                    m_ui->max_velocity_x->setEnabled(false);
                    m_ui->max_velocity_x_backwards->setEnabled(false);
                    m_ui->max_velocity_y->setEnabled(false);
                    m_ui->max_velocity_theta->setEnabled(false);
                    m_ui->acceleration_limit_x->setEnabled(false);
                    m_ui->acceleration_limit_y->setEnabled(false);
                    m_ui->acceleration_limit_theta->setEnabled(false);
                    m_ui->min_turning_radius->setEnabled(false);
                    m_ui->wheelbase->setEnabled(false);

                    // Goal Tolerance
                    m_ui->yaw_goal_tolerance->setEnabled(false);
                    m_ui->xy_goal_tolerance->setEnabled(false);
                    m_ui->free_goal_velocity->setEnabled(false);

                    // Obstacles
                    m_ui->min_obstacle_distance->setEnabled(false);
                    m_ui->inflation_distance->setEnabled(false);
                    m_ui->dynamic_obstacle_inflation_distance->setEnabled(false);
                    m_ui->include_dynamic_obstacles->setEnabled(false);
                    m_ui->obstacles_poses_affected->setEnabled(false);
                    m_ui->obstacle_association_force_inclusion_factor->setEnabled(false);
                    m_ui->obstacle_association_cutoff_factor->setEnabled(false);

                    // Optimization
                    m_ui->num_inner_iterations->setEnabled(false);
                    m_ui->num_outer_iterations->setEnabled(false);
                    m_ui->penalty_epsilon->setEnabled(false);
                    m_ui->max_velocity_x_weight->setEnabled(false);
                    m_ui->max_velocity_y_weight->setEnabled(false);
                    m_ui->max_velocity_theta_weight->setEnabled(false);
                    m_ui->acceleration_limit_x_weight->setEnabled(false);
                    m_ui->acceleration_limit_y_weight->setEnabled(false);
                    m_ui->acceleration_limit_theta_weight->setEnabled(false);
                    m_ui->kinematics_nh_weight->setEnabled(false);
                    m_ui->kinematics_forward_drive_weight->setEnabled(false);
                    m_ui->kinematics_turning_radius_weight->setEnabled(false);
                    m_ui->optimal_time_weight->setEnabled(false);
                    m_ui->obstacle_weight->setEnabled(false);
                    m_ui->inflation_weight->setEnabled(false);
                    m_ui->dynamic_obstacle_weight->setEnabled(false);
                    m_ui->dynamic_obstacle_inflation_weight->setEnabled(false);
                    m_ui->waypoint_weight->setEnabled(false);
                    m_ui->weight_adapt_factor->setEnabled(false);
                }
            }

            void ConfigWidget::removeConfig()
            {
                int index = m_ui->config_options->currentIndex();
                if (index >= 0)
                {
                    m_settings->removeConfig(m_name);
                    m_ui->config_options->removeItem(index);
                }
            }
        }
    }
}
