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

                // HCP
                m_ui->enable_hcp->setEnabled(false);
                m_ui->enable_mt->setEnabled(false);
                m_ui->simple_exploration->setEnabled(false);
                m_ui->max_number_classes->setEnabled(false);
                m_ui->selection_cost_hysteresis->setEnabled(false);
                m_ui->selection_obstacle_cost_scale->setEnabled(false);
                m_ui->selection_prefer_initial_plan->setEnabled(false);
                m_ui->selection_waypoint_cost_scale->setEnabled(false);
                m_ui->selection_alternative_time_cost->setEnabled(false);
                m_ui->roadmap_graph_no_samples->setEnabled(false);
                m_ui->roadmap_graph_area_width->setEnabled(false);
                m_ui->roadmap_graph_area_length_scale->setEnabled(false);
                m_ui->h_signature_prescaler->setEnabled(false);
                m_ui->h_signature_threshold->setEnabled(false);
                m_ui->obstacle_keypoint_offset->setEnabled(false);
                m_ui->obstacle_heading_threshold->setEnabled(false);
                m_ui->waypoint_all_candidates->setEnabled(false);


                // Trajectory
                setupMap(m_bools, autosize);
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

                // HCP
                setupMap(m_bools, enable_hcp);
                setupMap(m_bools, enable_mt);
                setupMap(m_bools, simple_exploration);
                setupMap(m_integers, max_number_classes);
                setupMap(m_doubles, selection_cost_hysteresis);
                setupMap(m_doubles, selection_prefer_initial_plan);
                setupMap(m_doubles, selection_obstacle_cost_scale);
                setupMap(m_doubles, selection_waypoint_cost_scale);
                setupMap(m_bools, selection_alternative_time_cost);
                setupMap(m_integers, roadmap_graph_no_samples);
                setupMap(m_doubles, roadmap_graph_area_width);
                setupMap(m_doubles, roadmap_graph_area_length_scale);
                setupMap(m_doubles, h_signature_prescaler);
                setupMap(m_doubles, h_signature_threshold);
                setupMap(m_doubles, obstacle_keypoint_offset);
                setupMap(m_doubles, obstacle_heading_threshold);
                setupMap(m_bools, waypoint_all_candidates);

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
                        m_current->trajectory.global_plan_viapoint_sep = dist.to(units::m);
                    }
                    else if (var_name == "max_global_plan_lookahead_distance")
                    {
                        m_current->trajectory.max_global_plan_lookahead_dist = dist.to(units::m);
                    }
                    else if (var_name == "force_reinit_new_goal_distance")
                    {
                        m_current->trajectory.force_reinit_new_goal_dist = dist.to(units::m);
                    }
                    else if (var_name == "min_turning_radius")
                    {
                        m_current->robot.min_turning_radius = dist.to(units::m);
                    }
                    else if (var_name == "wheelbase")
                    {
                        m_current->robot.wheelbase = dist.to(units::m);
                    }
                    else if (var_name == "xy_goal_tolerance")
                    {
                        m_current->goal_tolerance.xy_goal_tolerance = dist.to(units::m);
                    }
                    else if (var_name == "min_obstacle_distance")
                    {
                        m_current->obstacles.min_obstacle_dist = dist.to(units::m);
                    }
                    else if (var_name == "inflation_distance")
                    {
                        m_current->obstacles.inflation_dist = dist.to(units::m);
                    }
                    else if (var_name == "dynamic_obstacle_inflation_distance")
                    {
                        m_current->obstacles.dynamic_obstacle_inflation_dist = dist.to(units::m);
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
                        m_current->trajectory.dt_ref = time.to(units::s);
                    }
                    else if (var_name == "dt_hysteresis")
                    {
                        m_current->trajectory.dt_hysteresis = time.to(units::s);
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
                        m_current->robot.max_vel_x = vel.to(units::m / units::s);
                    }
                    else if (var_name == "max_velocity_x_backwards")
                    {
                        m_current->robot.max_vel_x_backwards = vel.to(units::m / units::s);;
                    }
                    else if (var_name == "max_velocity_y")
                    {
                        m_current->robot.max_vel_y = vel.to(units::m / units::s);;
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
                        m_current->robot.acc_lim_x = acc.to(units::m / units::s / units::s);;
                    }
                    else if (var_name == "acceleration_limit_y")
                    {
                        m_current->robot.acc_lim_y = acc.to(units::m / units::s / units::s);;
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
                        m_current->goal_tolerance.yaw_goal_tolerance = angle.to(units::rad);
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
                        m_current->robot.max_vel_theta = vel.to(units::rad / units::s);
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
                        m_current->robot.acc_lim_theta = acc.to(units::rad / units::s / units::s);
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
                    if (var_name == "obstacle_association_force_inclusion_factor")
                    {
                        m_current->obstacles.obstacle_association_force_inclusion_factor = value;
                    }
                    else if (var_name == "obstacle_association_cutoff_factor")
                    {
                        m_current->obstacles.obstacle_association_cutoff_factor = value;
                    }
                    else if (var_name == "penalty_epsilon")
                    {
                        m_current->optim.penalty_epsilon = value;
                    }
                    else if (var_name == "max_velocity_x_weight")
                    {
                        m_current->optim.weight_max_vel_x = value;
                    }
                    else if (var_name == "max_velocity_y_weight")
                    {
                        m_current->optim.weight_max_vel_y = value;
                    }
                    else if (var_name == "max_velocity_theta_weight")
                    {
                        m_current->optim.weight_max_vel_theta = value;
                    }
                    else if (var_name == "acceleration_limit_x_weight")
                    {
                        m_current->optim.weight_acc_lim_x = value;
                    }
                    else if (var_name == "acceleration_limit_y_weight")
                    {
                        m_current->optim.weight_acc_lim_y = value;
                    }
                    else if (var_name == "acceleration_limit_theta_weight")
                    {
                        m_current->optim.weight_acc_lim_theta = value;
                    }
                    else if (var_name == "kinematics_nh_weight")
                    {
                        m_current->optim.weight_kinematics_nh = value;
                    }
                    else if (var_name == "kinematics_forward_drive_weight")
                    {
                        m_current->optim.weight_kinematics_forward_drive = value;
                    }
                    else if (var_name == "kinematics_turning_radius_weight")
                    {
                        m_current->optim.weight_kinematics_turning_radius = value;
                    }
                    else if (var_name == "optimal_time_weight")
                    {
                        m_current->optim.weight_optimaltime = value;
                    }
                    else if (var_name == "obstacle_weight")
                    {
                        m_current->optim.weight_obstacle = value;
                    }
                    else if (var_name == "inflation_weight")
                    {
                        m_current->optim.weight_inflation = value;
                    }
                    else if (var_name == "dynamic_obstacle_weight")
                    {
                        m_current->optim.weight_dynamic_obstacle = value;
                    }
                    else if (var_name == "dynamic_obstacle_inflation_weight")
                    {
                        m_current->optim.weight_dynamic_obstacle_inflation = value;
                    }
                    else if (var_name == "waypoint_weight")
                    {
                        m_current->optim.weight_viapoint = value;
                    }
                    else if (var_name == "weight_adapt_factor")
                    {
                        m_current->optim.weight_adapt_factor = value;
                    }
                    else if(var_name == "selection_cost_hysteresis")
                    {
                        m_current->hcp.selection_cost_hysteresis = value;
                    }
                    else if(var_name == "selection_prefer_initial_plan")
                    {
                        m_current->hcp.selection_prefer_initial_plan = value;
                    }
                    else if(var_name == "selection_obstacle_cost_scale")
                    {
                        m_current->hcp.selection_obst_cost_scale = value;
                    }
                    else if(var_name == "selection_waypoint_cost_scale")
                    {
                        m_current->hcp.selection_viapoint_cost_scale = value;
                    }
                    else if(var_name == "roadmap_graph_area_width")
                    {
                        m_current->hcp.roadmap_graph_area_width = value;
                    }
                    else if(var_name == "roadmap_graph_area_length_scale")
                    {
                        m_current->hcp.roadmap_graph_area_length_scale = value;
                    }
                    else if(var_name == "h_signature_prescaler")
                    {
                        m_current->hcp.h_signature_prescaler = value;
                    }
                    else if(var_name == "h_signarture_threshold")
                    {
                        m_current->hcp.h_signature_threshold = value;
                    }
                    else if(var_name == "obstacle_keypoint_offset")
                    {
                        m_current->hcp.obstacle_keypoint_offset = value;
                    }
                    else if(var_name == "obstacle_heading_threshold")
                    {
                        m_current->hcp.obstacle_heading_threshold = value;
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
                        m_current->obstacles.obstacle_poses_affected = value;
                    }
                    else if (var_name == "num_inner_iterations")
                    {
                        m_current->optim.no_inner_iterations = value;
                    }
                    else if (var_name == "num_outer_iterations")
                    {
                        m_current->optim.no_outer_iterations = value;
                    }
                    else if(var_name == "max_number_classes")
                    {
                        m_current->hcp.max_number_classes = value;
                    }
                    else if(var_name == "roadmap_graph_no_samples")
                    {
                        m_current->hcp.roadmap_graph_no_samples = value;
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
                if(var_name == "autosize")
                {
                    m_current->trajectory.teb_autosize = value;
                }
                else if (var_name == "global_plan_overwrite_orientation")
                {
                    m_current->trajectory.global_plan_overwrite_orientation = value;
                }
                else if (var_name == "allow_init_with_backwards_motion")
                {
                    m_current->trajectory.allow_init_with_backwards_motion = value;
                }
                else if (var_name == "waypoints_ordered")
                {
                    m_current->trajectory.via_points_ordered = value;
                }
                else if (var_name == "exact_arc_length")
                {
                    m_current->trajectory.exact_arc_length = value;
                }
                else if (var_name == "free_goal_velocity")
                {
                    m_current->goal_tolerance.free_goal_vel = value;
                }
                else if (var_name == "include_dynamic_obstacles")
                {
                    m_current->obstacles.include_dynamic_obstacles = value;
                }
                else if(var_name == "enable_hcp")
                {
                    m_current->hcp.enable_homotopy_class_planning = value;
                }
                else if(var_name == "enable_mt")
                {
                    m_current->hcp.enable_multithreading = value;
                }
                else if(var_name == "simple_exploration")
                {
                    m_current->hcp.simple_exploration = value;
                }
                else if(var_name == "selection_alternative_time_cost")
                {
                    m_current->hcp.selection_alternative_time_cost = value;
                }
                else if(var_name == "waypoint_all_candidates")
                {
                    m_current->hcp.viapoints_all_candidates = value;
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

                    // HCP
                    m_ui->enable_hcp->setEnabled(true);
                    m_ui->enable_mt->setEnabled(true);
                    m_ui->simple_exploration->setEnabled(true);
                    m_ui->max_number_classes->setEnabled(true);
                    m_ui->selection_cost_hysteresis->setEnabled(true);
                    m_ui->selection_obstacle_cost_scale->setEnabled(true);
                    m_ui->selection_prefer_initial_plan->setEnabled(true);
                    m_ui->selection_waypoint_cost_scale->setEnabled(true);
                    m_ui->selection_alternative_time_cost->setEnabled(true);
                    m_ui->roadmap_graph_no_samples->setEnabled(true);
                    m_ui->roadmap_graph_area_width->setEnabled(true);
                    m_ui->roadmap_graph_area_length_scale->setEnabled(true);
                    m_ui->h_signature_prescaler->setEnabled(true);
                    m_ui->h_signature_threshold->setEnabled(true);
                    m_ui->obstacle_keypoint_offset->setEnabled(true);
                    m_ui->obstacle_heading_threshold->setEnabled(true);
                    m_ui->waypoint_all_candidates->setEnabled(true);

                    // Set

                    // Trajectory
                    m_ui->autosize->setChecked(m_current->trajectory.teb_autosize);
                    m_ui->dt_ref->setText(QString::number(m_current->trajectory.dt_ref, 'f', 2));
                    m_ui->dt_hysteresis->setText(QString::number(m_current->trajectory.dt_hysteresis, 'f', 2));
                    m_ui->min_samples->setText(QString::number(m_current->trajectory.min_samples));
                    m_ui->max_samples->setText(QString::number(m_current->trajectory.max_samples));
                    m_ui->overwrite_global_plan->setChecked(m_current->trajectory.global_plan_overwrite_orientation);
                    m_ui->backwards_motion->setChecked(m_current->trajectory.allow_init_with_backwards_motion);
                    m_ui->global_plan_waypoint_separation->setText(QString::number((m_current->trajectory.global_plan_viapoint_sep * units::m).to(units::in), 'f', 2));
                    m_ui->waypoints_ordered->setChecked(m_current->trajectory.via_points_ordered);
                    m_ui->max_global_plan_lookahead_distance->setText(QString::number((m_current->trajectory.max_global_plan_lookahead_dist * units::m).to(units::in), 'f', 2));
                    m_ui->exact_arc_length->setChecked(m_current->trajectory.exact_arc_length);
                    m_ui->force_new_goal_distance->setText(QString::number((m_current->trajectory.force_reinit_new_goal_dist * units::m).to(units::in), 'f', 2));
                    m_ui->feasibility_check_num_poses->setText(QString::number(m_current->trajectory.feasibility_check_no_poses));

                    // Robot
                    m_ui->max_velocity_x->setText(QString::number((m_current->robot.max_vel_x * units::m / units::s).to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_x_backwards->setText(QString::number((m_current->robot.max_vel_x_backwards * units::m / units::s).to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_y->setText(QString::number((m_current->robot.max_vel_y * units::m / units::s).to(units::in / units::s), 'f', 2));
                    m_ui->max_velocity_theta->setText(QString::number((m_current->robot.max_vel_theta * units::rad / units::s).to(units::deg / units::s), 'f', 2));
                    m_ui->acceleration_limit_x->setText(QString::number((m_current->robot.acc_lim_x * units::m / units::s / units::s).to(units::in / units::s / units::s), 'f', 2));
                    m_ui->acceleration_limit_y->setText(QString::number((m_current->robot.acc_lim_y * units::m / units::s / units::s).to(units::in / units::s / units::s), 'f', 2));
                    m_ui->acceleration_limit_theta->setText(QString::number((m_current->robot.acc_lim_theta * units::rad / units::s / units::s).to(units::deg / units::s / units::s), 'f', 2));
                    m_ui->min_turning_radius->setText(QString::number((m_current->robot.min_turning_radius * units::m).to(units::in), 'f', 2));
                    m_ui->wheelbase->setText(QString::number((m_current->robot.wheelbase * units::m).to(units::in), 'f', 2));

                    // Goal Tolerance
                    m_ui->yaw_goal_tolerance->setText(QString::number((m_current->goal_tolerance.yaw_goal_tolerance * units::rad).to(units::deg), 'f', 2));
                    m_ui->xy_goal_tolerance->setText(QString::number((m_current->goal_tolerance.xy_goal_tolerance * units::m).to(units::in), 'f', 2));
                    m_ui->free_goal_velocity->setChecked(m_current->goal_tolerance.free_goal_vel);

                    // Obstacles
                    m_ui->min_obstacle_distance->setText(QString::number((m_current->obstacles.min_obstacle_dist * units::m).to(units::in), 'f', 2));
                    m_ui->inflation_distance->setText(QString::number((m_current->obstacles.inflation_dist * units::m).to(units::in), 'f', 2));
                    m_ui->dynamic_obstacle_inflation_distance->setText(QString::number((m_current->obstacles.dynamic_obstacle_inflation_dist * units::m).to(units::in), 'f', 2));
                    m_ui->include_dynamic_obstacles->setChecked(m_current->obstacles.include_dynamic_obstacles);
                    m_ui->obstacles_poses_affected->setText(QString::number(m_current->obstacles.obstacle_poses_affected));
                    m_ui->obstacle_association_force_inclusion_factor->setText(QString::number(m_current->obstacles.obstacle_association_force_inclusion_factor, 'f', 2));
                    m_ui->obstacle_association_cutoff_factor->setText(QString::number(m_current->obstacles.obstacle_association_cutoff_factor, 'f', 2));

                    // Optimization
                    m_ui->num_inner_iterations->setText(QString::number(m_current->optim.no_inner_iterations));
                    m_ui->num_outer_iterations->setText(QString::number(m_current->optim.no_outer_iterations));
                    m_ui->penalty_epsilon->setText(QString::number(m_current->optim.penalty_epsilon, 'f', 2));
                    m_ui->max_velocity_x_weight->setText(QString::number(m_current->optim.weight_max_vel_x, 'f', 2));
                    m_ui->max_velocity_y_weight->setText(QString::number(m_current->optim.weight_max_vel_y, 'f', 2));
                    m_ui->max_velocity_theta_weight->setText(QString::number(m_current->optim.weight_max_vel_theta, 'f', 2));
                    m_ui->acceleration_limit_x_weight->setText(QString::number(m_current->optim.weight_acc_lim_x, 'f', 2));
                    m_ui->acceleration_limit_y_weight->setText(QString::number(m_current->optim.weight_acc_lim_y, 'f', 2));
                    m_ui->acceleration_limit_theta_weight->setText(QString::number(m_current->optim.weight_acc_lim_theta, 'f', 2));
                    m_ui->kinematics_nh_weight->setText(QString::number(m_current->optim.weight_kinematics_nh, 'f', 2));
                    m_ui->kinematics_forward_drive_weight->setText(QString::number(m_current->optim.weight_kinematics_forward_drive, 'f', 2));
                    m_ui->kinematics_turning_radius_weight->setText(QString::number(m_current->optim.weight_kinematics_turning_radius, 'f', 2));
                    m_ui->optimal_time_weight->setText(QString::number(m_current->optim.weight_optimaltime, 'f', 2));
                    m_ui->obstacle_weight->setText(QString::number(m_current->optim.weight_obstacle, 'f', 2));
                    m_ui->inflation_weight->setText(QString::number(m_current->optim.weight_inflation, 'f', 2));
                    m_ui->dynamic_obstacle_weight->setText(QString::number(m_current->optim.weight_dynamic_obstacle, 'f', 2));
                    m_ui->dynamic_obstacle_inflation_weight->setText(QString::number(m_current->optim.weight_dynamic_obstacle_inflation, 'f', 2));
                    m_ui->waypoint_weight->setText(QString::number(m_current->optim.weight_viapoint, 'f', 2));
                    m_ui->weight_adapt_factor->setText(QString::number(m_current->optim.weight_adapt_factor, 'f', 2));

                    // HCP
                    m_ui->enable_hcp->setChecked(m_current->hcp.enable_homotopy_class_planning);
                    m_ui->enable_mt->setChecked(m_current->hcp.enable_multithreading);
                    m_ui->simple_exploration->setChecked(m_current->hcp.simple_exploration);
                    m_ui->max_number_classes->setText(QString::number(m_current->hcp.max_number_classes));
                    m_ui->selection_cost_hysteresis->setText(QString::number(m_current->hcp.selection_cost_hysteresis, 'f', 2));
                    m_ui->selection_obstacle_cost_scale->setText(QString::number(m_current->hcp.selection_obst_cost_scale, 'f', 2));
                    m_ui->selection_prefer_initial_plan->setText(QString::number(m_current->hcp.selection_prefer_initial_plan, 'f', 2));
                    m_ui->selection_waypoint_cost_scale->setText(QString::number(m_current->hcp.selection_viapoint_cost_scale, 'f', 2));
                    m_ui->selection_alternative_time_cost->setText(QString::number(m_current->hcp.selection_alternative_time_cost, 'f', 2));
                    m_ui->roadmap_graph_no_samples->setText(QString::number(m_current->hcp.roadmap_graph_no_samples, 'f', 2));
                    m_ui->roadmap_graph_area_width->setText(QString::number(m_current->hcp.roadmap_graph_area_width, 'f', 2));
                    m_ui->roadmap_graph_area_length_scale->setText(QString::number(m_current->hcp.roadmap_graph_area_length_scale, 'f', 2));
                    m_ui->h_signature_prescaler->setText(QString::number(m_current->hcp.h_signature_prescaler, 'f', 2));
                    m_ui->h_signature_threshold->setText(QString::number(m_current->hcp.h_signature_threshold, 'f', 2));
                    m_ui->obstacle_keypoint_offset->setText(QString::number(m_current->hcp.obstacle_keypoint_offset, 'f', 2));
                    m_ui->obstacle_heading_threshold->setText(QString::number(m_current->hcp.obstacle_heading_threshold, 'f', 2));
                    m_ui->waypoint_all_candidates->setChecked(m_current->hcp.viapoints_all_candidates);
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

                    // HCP
                    m_ui->enable_hcp->setEnabled(false);
                    m_ui->enable_mt->setEnabled(false);
                    m_ui->simple_exploration->setEnabled(false);
                    m_ui->max_number_classes->setEnabled(false);
                    m_ui->selection_cost_hysteresis->setEnabled(false);
                    m_ui->selection_obstacle_cost_scale->setEnabled(false);
                    m_ui->selection_prefer_initial_plan->setEnabled(false);
                    m_ui->selection_waypoint_cost_scale->setEnabled(false);
                    m_ui->selection_alternative_time_cost->setEnabled(false);
                    m_ui->roadmap_graph_no_samples->setEnabled(false);
                    m_ui->roadmap_graph_area_width->setEnabled(false);
                    m_ui->roadmap_graph_area_length_scale->setEnabled(false);
                    m_ui->h_signature_prescaler->setEnabled(false);
                    m_ui->h_signature_threshold->setEnabled(false);
                    m_ui->obstacle_keypoint_offset->setEnabled(false);
                    m_ui->obstacle_heading_threshold->setEnabled(false);
                    m_ui->waypoint_all_candidates->setEnabled(false);
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
