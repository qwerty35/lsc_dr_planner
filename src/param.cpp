#include <param.hpp>
#define GET_VARIABLE_NAME(Variable) (#Variable)

namespace DynamicPlanning {
    bool Param::initialize(const ros::NodeHandle &nh) {
        nh.param<bool>("log_solver", log_solver, false);
        nh.param<bool>("log_vis", log_vis, true);

        // World
        nh.param<std::string>("world/frame_id", world_frame_id, "world");
        nh.param<int>("world/dimension", world_dimension, 3);
        nh.param<bool>("world/use_octomap", world_use_octomap, false);
        nh.param<double>("world/resolution", world_resolution, 0.1);
        nh.param<double>("world/z_2d", world_z_2d, 1.0);
        nh.param<bool>("world/use_global_map", world_use_global_map, true);
        nh.param<double>("world/max_dist", world_max_dist, 1.0);

        // Multisim setting
        nh.param<int>("multisim/planning_rate", multisim_planning_rate, -1);
        nh.param<int>("multisim/qn", multisim_qn, 2);
        nh.param<double>("multisim/time_step", multisim_time_step, 0.1);
        nh.param<bool>("multisim/patrol", multisim_patrol, false);
        nh.param<double>("multisim/max_noise", multisim_max_noise, 0.0);
        nh.param<int>("multisim/max_planner_iteration", multisim_max_planner_iteration, 1000);
        nh.param<bool>("multisim/save_result", multisim_save_result, false);
        nh.param<bool>("multisim/save_mission", multisim_save_mission, false);
        nh.param<bool>("multisim/replay", multisim_replay, false);
        nh.param<std::string>("multisim/replay_file_name", multisim_replay_file_name, "default.csv");
        nh.param<double>("multisim/replay_time_limit", multisim_replay_time_limit, -1);
        nh.param<double>("multisim/record_time_step", multisim_save_time_step, 0.1);

        // Goal mode
        std::string goal_mode_str;
        nh.param<std::string>("mode/goal", goal_mode_str, "prior_based");
        if (goal_mode_str == "right_hand") {
            goal_mode = GoalMode::RIGHTHAND;
        } else if (goal_mode_str == "prior_based") {
            goal_mode = GoalMode::PRIORBASED;
        } else if (goal_mode_str == "dynamic_priority") {
            goal_mode = GoalMode::DYNAMICPRIORITY;
        } else if (goal_mode_str == "entropy") {
            goal_mode = GoalMode::ENTROPY;
        } else if (goal_mode_str == "grid_based_planner") {
            goal_mode = GoalMode::GRIDBASEDPLANNER;
        } else {
            ROS_ERROR("[Param] Invalid goal mode");
            return false;
        }

        // MAPF mode
        std::string mapf_mode_str;
        nh.param<std::string>("mode/mapf", mapf_mode_str, "pibt");
        if (mapf_mode_str == "pibt") {
            mapf_mode = MAPFMode::PIBT;
        } else if (mapf_mode_str == "ecbs") {
            mapf_mode = MAPFMode::ECBS;
        } else {
            ROS_ERROR("[Param] Invalid mapf mode");
            return false;
        }

        // Obstacle prediction
        nh.param<bool>("obs/size_prediction", obs_size_prediction, true);
        nh.param<double>("obs/uncertainty_horizon", obs_uncertainty_horizon, 1);
        nh.param<bool>("obs/agent_clustering", obs_agent_clustering, false);
        nh.param<bool>("obs/use_velocity_guard", use_velocity_guard, true);
        nh.param<double>("obs/velocity_guard_ratio", velocity_guard_ratio, 0.75);

        // Trajectory representation
        nh.param<double>("traj/dt", dt, 0.2);
        nh.param<int>("traj/M", M, 5);
        nh.param<int>("traj/n", n, 5);
        nh.param<int>("traj/phi", phi, 3);
        nh.param<int>("traj/phi_n", phi_n, 1);

        // Trajectory optimization
        nh.param<double>("opt/control_input_weight", control_input_weight, 1);
        nh.param<double>("opt/terminal_weight", terminal_weight, 1);
        nh.param<double>("opt/slack_collision_weight", slack_collision_weight, 1);
        nh.param<double>("opt/slack_dynamic_weight", slack_dynamic_weight, 1);

        // Deadlock
        nh.param<double>("deadlock/velocity_threshold", deadlock_velocity_threshold, 0.1);
        nh.param<int>("deadlock/seq_threshold", deadlock_seq_threshold, 5);

        // Filter
        nh.param<double>("filter/sigma_y_sq", filter_sigma_y_sq, 0.0036);
        nh.param<double>("filter/sigma_v_sq", filter_sigma_v_sq, 0.01);
        nh.param<double>("filter/sigma_a_sq", filter_sigma_a_sq, 1.0);

        // ORCA
        nh.param<double>("orca/horizon", orca_horizon, 2.0);
        nh.param<double>("orca/pref_velocity_ratio", ocra_pref_velocity_ratio, 1.0);
        nh.param<double>("orca/inflation_ratio", orca_inflation_ratio, 1.5);

        // Grid-based planner
        nh.param<double>("grid/resolution", grid_resolution, 0.3);
        nh.param<double>("grid/margin", grid_margin, 0.1);

        // Goal
        nh.param<double>("plan/goal_threshold", goal_threshold, 0.1);
        nh.param<double>("plan/goal_radius", goal_radius, 100.0);
        nh.param<double>("plan/priority_agent_distance", priority_agent_distance, 0.1);
        nh.param<double>("plan/priority_obs_distance", priority_obs_distance, 1.0);
        nh.param<double>("plan/priority_goal_threshold", priority_goal_threshold, 0.6);
        nh.param<double>("plan/reset_threshold", reset_threshold, 0.1);
        nh.param<double>("plan/slack_threshold", slack_threshold, 0.001);
        nh.param<double>("plan/obs_downwash_threshold", obs_downwash_threshold, 3.0);
        nh.param<double>("plan/collision_alert_threshold", collision_alert_threshold, 1.0);
        nh.param<double>("plan/density_alert_threshold", density_alert_threshold, 0.001);
        nh.param<double>("plan/closest_agent_threshold", closest_agent_threshold, 0.1);

        // SFC
        nh.param<double>("plan/numerical_error_threshold", numerical_error_threshold, 0.01);

        // Communication
        nh.param<double>("communication/range", communication_range, 3.0);

        // Exploration
        nh.param<double>("sensor/range", sensor_range, 3.0);

        // Debug
        nh.param<int>("debug/planner_seq", debug_planner_seq, 0);

        package_path = ros::package::getPath("lsc_dr_planner");

        // Planner mode
        std::string planner_mode_str;
        nh.param<std::string>("mode/planner", planner_mode_str, "dlsc");
        if (planner_mode_str == "dlsc") {
            planner_mode = PlannerMode::DLSC;
            prediction_mode = PredictionMode::PREVIOUSSOLUTION;
            initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
            if (multisim_time_step < dt) {
                slack_mode = SlackMode::CONTINUITY;
            } else if (multisim_time_step == dt) {
                slack_mode = SlackMode::NONE;
            } else {
                ROS_ERROR("[Param] Invalid parameter, multisim_time_step > dt");
            }
        } else if (planner_mode_str == "lsc") {
            planner_mode = PlannerMode::LSC;
            prediction_mode = PredictionMode::PREVIOUSSOLUTION;
            initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
            if (multisim_time_step == dt) {
                slack_mode = SlackMode::NONE;
            } else {
                ROS_ERROR("[Param] Invalid parameter, multisim_time_step != dt");
            }
        } else if (planner_mode_str == "bvc") {
            planner_mode = PlannerMode::BVC;
            prediction_mode = PredictionMode::POSITION;
            initial_traj_mode = InitialTrajMode::POSITION;
            slack_mode = SlackMode::NONE;
        } else if (planner_mode_str == "orca") {
            planner_mode = PlannerMode::ORCA;
        } else if (planner_mode_str == "reciprocal_rsfc") {
            planner_mode = PlannerMode::RECIPROCALRSFC;
            prediction_mode = PredictionMode::VELOCITY;
            initial_traj_mode = InitialTrajMode::ORCA;
            slack_mode = SlackMode::COLLISIONCONSTRAINT;
        } else if (planner_mode_str == "circle_test") {
            planner_mode = PlannerMode::CIRCLETEST;
            prediction_mode = PredictionMode::VELOCITY;
            initial_traj_mode = InitialTrajMode::VELOCITY;
            slack_mode = SlackMode::NONE;
        } else {
            ROS_ERROR("[Param] Invalid planner mode");
            return false;
        }

        return true;
    }

    std::string Param::getPlannerModeStr() const {
        const std::string planner_mode_strs[] = {"DLSC", "LSC", "BVC", "ORCA", "ReciprocalRSFC", "CircleTest"};
        return planner_mode_strs[static_cast<int>(planner_mode)];
    }

    std::string Param::getPredictionModeStr() const {
        const std::string prediction_mode_strs[] = {"current_position", "constant_velocity", "orca",
                                                    "previous_solution"};
        return prediction_mode_strs[static_cast<int>(prediction_mode)];
    }

    std::string Param::getInitialTrajModeStr() const {
        const std::string initial_traj_mode_strs[] = {"current_position", "current_velocity", "orca",
                                                      "previous_solution", "skip"};
        return initial_traj_mode_strs[static_cast<int>(initial_traj_mode)];
    }

    std::string Param::getSlackModeStr() const {
        const std::string slack_mode_strs[] = {"none", "dynamical_limit", "collision_constraint"};
        return slack_mode_strs[static_cast<int>(slack_mode)];
    }

    std::string Param::getGoalModeStr() const {
        const std::string planner_mode_strs[] = {"static", "orca", "right_hand",
                                                 "prior_based", "dynamic_priority",
                                                 "entropy", "grid_based_planner"};
        return planner_mode_strs[static_cast<int>(goal_mode)];
    }

    std::string Param::getMAPFModeStr() const {
        const std::string planner_mode_strs[] = {"pibt", "ecbs"};
        return planner_mode_strs[static_cast<int>(mapf_mode)];
    }
}