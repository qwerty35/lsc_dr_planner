#include <traj_planner.hpp>

namespace DynamicPlanning {
    TrajPlanner::TrajPlanner(const ros::NodeHandle &_nh,
                             const Param &_param,
                             const Mission &_mission,
                             const Agent &_agent)
            : nh(_nh), param(_param), mission(_mission), constraints(_param, _mission), agent(_agent) {
        // Initialize useful constants
        buildBernsteinBasis(param.n, B, B_inv);

        // Initialize planner state
        planner_seq = 0;
        is_disturbed = false;
        is_sol_converged_by_sfc = false;
        goal_planner_state = GoalPlannerState::FORWARD;
        initialize_sfc = false;
        desired_segment_idx = param.M - 1;
        if(param.planner_mode == PlannerMode::LSC){
            initialize_sfc = true;
        }

        // Initialize grid based planner module
        grid_based_planner = std::make_unique<GridBasedPlanner>(param, mission);

        // Initialize trajectory optimization module
        traj_optimizer = std::make_unique<TrajOptimizer>(param, mission, B);

        // Initialize ROS
        initializeROS();
    }

    traj_t TrajPlanner::plan(const Agent &_agent,
                             const std::shared_ptr <octomap::OcTree> &_octree_ptr,
                             const std::shared_ptr <DynamicEDTOctomap> &_distmap_ptr,
                             ros::Time _sim_current_time,
                             bool _is_disburbed) {
        // Initialize planner
        ros::Time planning_start_time = ros::Time::now();
        agent = _agent;
        sim_current_time = _sim_current_time;
        octree_ptr = _octree_ptr;
        distmap_ptr = _distmap_ptr;
        constraints.setDistmap(distmap_ptr);
        constraints.setOctomap(octree_ptr);
        is_disturbed = _is_disburbed;

        // Start planning
        planner_seq++;
        statistics.planning_seq = planner_seq;
        traj_t desired_traj = planImpl();

        // Re-initialization for replanning
        prev_traj = desired_traj;

        // Print terminal message
        statistics.planning_time.total_planning_time.update((ros::Time::now() - planning_start_time).toSec());

        return desired_traj;
    }

    void TrajPlanner::publish() {
        if(param.log_vis){
//            publishInitialTraj();
//            publishCollisionConstraints();
//            publishGridOccupiedPoints();
            publishFeasibleRegion();
            publishGridPath();
        }
        publishObstaclePrediction();
        publishSFC();
        publishLSC();
    }

    void TrajPlanner::setObstacles(const std::vector<Obstacle> &msg_obstacles) {
        obstacles = msg_obstacles;
    }

    int TrajPlanner::getPlannerSeq() const {
        return planner_seq;
    }

    point3d TrajPlanner::getCurrentGoalPosition() const {
        return agent.current_goal_point;
    }

    PlanningStatistics TrajPlanner::getPlanningStatistics() const {
        return statistics;
    }

    bool TrajPlanner::getCollisionAlert() const {
        return not col_pred_obs_indices.empty();
    }

//    double TrajPlanner::getSlackVelCont() const {
//        return error_vel_continuity;
//    }
//
//    double TrajPlanner::getSlackAccCont() const {
//        return error_acc_continuity;
//    }

    void TrajPlanner::initializeROS() {
        // Initialize ros publisher and subscriber
        std::string prefix = "/mav" + std::to_string(agent.id);
        pub_sfc = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/sfc", 1);
        pub_lsc = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/lsc", 1);
        pub_feasible_region = nh.advertise<visualization_msgs::MarkerArray>("/feasible_region", 1);
        pub_initial_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "/initial_traj_vis", 1);
        pub_obs_pred_traj_vis = nh.advertise<visualization_msgs::MarkerArray>("/obs_pred_traj_vis", 1);
        pub_grid_path = nh.advertise<visualization_msgs::MarkerArray>("/grid_path_vis", 1);
        pub_grid_occupied_points = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "/grid_occupied_points", 1);
    }

    traj_t TrajPlanner::planImpl() {
        // Check the current planner mode is valid.
        checkPlannerMode();

        // Plan initial trajectory of the other agents
        obstaclePrediction();

        // Plan initial trajectory of current agent.
        initialTrajPlanning();

        // Construct LSC or BVC
        constructLSC();

        // construct SFC
        constructSFC();

        // Goal planning
        goalPlanning();

        // Trajectory optimization
        traj_t desired_traj = trajOptimization();
        return desired_traj;
    }

    void TrajPlanner::checkPlannerMode() {
        // Check planner mode at the first iteration only
        if (planner_seq > 1) {
            return;
        }

        switch (param.planner_mode) {
            case PlannerMode::DLSC:
                if (param.multisim_time_step > param.dt) {
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be smaller than segment time");
                }
                else if(param.multisim_time_step == param.dt and param.slack_mode != SlackMode::NONE){
                    ROS_WARN("[TrajPlanner] DLSC does not need slack variables when multisim_time_step == segment time, fix SlackMode to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                else if(param.multisim_time_step < param.dt and param.slack_mode != SlackMode::CONTINUITY){
                    ROS_WARN("[TrajPlanner] DLSC requires slack variables when multisim_time_step < segment time, fix SlackMode to dynamical_limit");
                    param.slack_mode = SlackMode::CONTINUITY;
                    traj_optimizer->updateParam(param);
                }

                if (param.prediction_mode != PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of LSC must be previous_solution, auto fixed");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if (param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be previous_solution, , auto fixed");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                break;
            case PlannerMode::LSC:
                if (param.multisim_time_step != param.dt) {
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to the segment time");
                }
                if (param.prediction_mode != PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of LSC must be previous_solution, fix automatically");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if (param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be previous_solution, fix automatically");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                if(param.slack_mode != SlackMode::NONE){
                    ROS_WARN("[TrajPlanner] LSC does not need slack variables, fix to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                break;
            case PlannerMode::BVC:
                if (param.prediction_mode != PredictionMode::POSITION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of BVC must be current_position, fix automatically");
                    param.prediction_mode = PredictionMode::POSITION;
                }
                if (param.initial_traj_mode != InitialTrajMode::POSITION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be current_position, fix automatically");
                    param.initial_traj_mode = InitialTrajMode::POSITION;
                }
                break;
            case PlannerMode::ORCA:
                break;
            case PlannerMode::RECIPROCALRSFC:
                if (param.prediction_mode == PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of Reciprocal RSFC must be current_position, fix to velocity automatically");
                    param.prediction_mode = PredictionMode::VELOCITY;
                }
                if (param.initial_traj_mode == InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of Reciprocal RSFC must be current_position, fix to ORCA automatically");
                    param.initial_traj_mode = InitialTrajMode::ORCA;
                }
                if (param.slack_mode != SlackMode::COLLISIONCONSTRAINT) {
                    ROS_WARN("[TrajPlanner] Reciprocal RSFC needs slack variables at collision constraints");
                    param.slack_mode = SlackMode::COLLISIONCONSTRAINT;
                    traj_optimizer->updateParam(param);
                }
                break;
        }

        if (param.world_use_octomap and distmap_ptr == nullptr) {
            throw std::invalid_argument("[TrajPlanner] Distmap is not ready");
        }
    }


    void TrajPlanner::obstaclePrediction() {
        // Timer start
        ros::Time obs_pred_start_time = ros::Time::now();

        // Initialize obstacle predicted trajectory
        size_t N_obs = obstacles.size();
        obs_pred_trajs.resize(N_obs);
        obs_pred_sizes.resize(N_obs);

        switch (param.prediction_mode) {
            case PredictionMode::POSITION:
                obstaclePredictionWithCurrPos();
                break;
            case PredictionMode::VELOCITY:
                obstaclePredictionWithCurrVel();
                break;
            case PredictionMode::PREVIOUSSOLUTION:
                obstaclePredictionWithPrevSol();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid obstacle prediction mode");
        }

        checkObstacleDisturbance();
        obstacleSizePredictionWithConstAcc();

        // Timer end
        ros::Time obs_pred_end_time = ros::Time::now();
        statistics.planning_time.obstacle_prediction_time.update((obs_pred_end_time - obs_pred_start_time).toSec());
    }

    void TrajPlanner::obstaclePredictionWithCurrPos() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);
            obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, point3d(0, 0, 0));
        }
    }

    void TrajPlanner::obstaclePredictionWithCurrVel() {
        // Obstacle prediction with constant velocity assumption
        // It assumes that correct position and velocity are given
        size_t N_obs = obstacles.size();
        for (size_t oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);
            obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, obstacles[oi].velocity);
        }
    }

    void TrajPlanner::obstaclePredictionWithPrevSol() {
        // Dynamic obstacle -> constant velocity, Agent -> prev sol
        // Use current velocity to predict the obstacle while the first iteration
        if (planner_seq < 2) {
            obstaclePredictionWithCurrVel();
            return;
        }

        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);

            if (obstacles[oi].type != ObstacleType::AGENT) {
                // if the obstacle is not agent, use current velocity to predict trajectory
                obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, obstacles[oi].velocity);
            } else if (param.multisim_time_step == param.dt) {
                // feasible LSC: generate C^n-continuous LSC
                for (int m = 0; m < param.M; m++) {
                    if (m == param.M - 1) {
                        for (int i = 0; i < param.n + 1; i++) {
                            obs_pred_trajs[oi][m][i] = obstacles[oi].prev_traj[m][param.n];
                        }
                    } else {
                        obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m + 1];
                    }
                }
            } else if (param.multisim_time_step < param.dt) {
                // relaxed LSC: generate C^0-continuous LSC
                for (int m = 0; m < param.M; m++) {
                    obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m];
                    if (m == 0) {
                        obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m].subSegment(param.multisim_time_step / param.dt, 1, B, B_inv);
                    }
                }
            } else {
                throw std::invalid_argument("[TrajPlanner] obs_pred_prev_sol supports only LSC, LSC2");
            }
        }
    }

    void TrajPlanner::checkObstacleDisturbance() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            point3d obs_position = obstacles[oi].position;
            if ((obs_pred_trajs[oi].startPoint() - obs_position).norm() > param.reset_threshold) {
                obs_pred_trajs[oi].planConstVelTraj(obs_position, point3d(0, 0, 0));
            }
        }
    }

    void TrajPlanner::obstacleSizePredictionWithConstAcc() {
        int M_uncertainty = static_cast<int>((param.obs_uncertainty_horizon + SP_EPSILON) / param.dt);
        double velocity_guard = 0;
        if (param.use_velocity_guard) {
            velocity_guard =
                    param.velocity_guard_ratio * (agent.current_state.velocity.norm_sq()) / agent.max_acc[0];
        }

        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_sizes[oi] = Trajectory<double>(param.M, param.n, param.dt);
            if (param.obs_size_prediction and (param.planner_mode == PlannerMode::RECIPROCALRSFC or
                                               obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE)) {
                // Predict obstacle size using max acc
                double max_acc;
                max_acc = obstacles[oi].max_acc;
                for (int m = 0; m < M_uncertainty; m++) {
                    Eigen::MatrixXd coef = Eigen::MatrixXd::Zero(1, param.n + 1);
                    coef(0, 0) = 0.5 * max_acc * pow(m * param.dt, 2);
                    coef(0, 1) = max_acc * m * param.dt * param.dt;
                    coef(0, 2) = 0.5 * max_acc * pow(param.dt, 2);

                    Eigen::MatrixXd control_points = coef * B_inv;
                    for (int i = 0; i < param.n + 1; i++) {
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius + velocity_guard + control_points(0, i);
                    }
                }
                for (int m = M_uncertainty; m < param.M; m++) {
                    for (int i = 0; i < param.n + 1; i++) {
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius +
                                                   velocity_guard +
                                                   0.5 * max_acc * pow(M_uncertainty * param.dt, 2);
                    }
                }
            } else {
                obs_pred_sizes[oi].planConstVelTraj(obstacles[oi].radius, 0);
            }
        }
    }

    void TrajPlanner::initialTrajPlanning() {
        // Timer start
        ros::Time init_traj_planning_start_time = ros::Time::now();

        initial_traj = Trajectory<point3d>(param.M, param.n, param.dt);
        switch (param.initial_traj_mode) {
            case InitialTrajMode::POSITION:
                initialTrajPlanningCurrPos();
                break;
            case InitialTrajMode::VELOCITY:
                initialTrajPlanningCurrVel();
                break;
            case InitialTrajMode::PREVIOUSSOLUTION:
                initialTrajPlanningPrevSol();
                break;
            case InitialTrajMode::SKIP:
                // skip
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid initial traj_curr planner mode");
        }

        // Check whether the agent is disturbed
        initialTrajPlanningCheck();

        // Timer end
        ros::Time init_traj_planning_end_time = ros::Time::now();
        statistics.planning_time.initial_traj_planning_time.update(
                (init_traj_planning_end_time - init_traj_planning_start_time).toSec());
    }

    void TrajPlanner::initialTrajPlanningCurrPos() {
        initial_traj.planConstVelTraj(agent.current_state.position, point3d(0, 0, 0));
    }

    void TrajPlanner::initialTrajPlanningCurrVel() {
        initial_traj.planConstVelTraj(agent.current_state.position, agent.current_state.velocity);
    }

    void TrajPlanner::initialTrajPlanningPrevSol() {
        if (planner_seq < 2) {
            initialTrajPlanningCurrVel();
        } else if (param.multisim_time_step == param.dt) {
            for (int m = 0; m < param.M; m++) {
                if (m == param.M - 1) {
                    for (int i = 0; i < param.n + 1; i++) {
                        initial_traj[m][i] = prev_traj[m][param.n];
                    }
                } else {
                    initial_traj[m] = prev_traj[m + 1];
                }
            }
        } else if (param.multisim_time_step < param.dt) {
            for (int m = 0; m < param.M; m++) {
                if (m == 0) {
                    initial_traj[m] = prev_traj[m].subSegment(param.multisim_time_step / param.dt, 1, B, B_inv);
                    initial_traj[m].segment_time = param.dt;
                }
                else{
                    initial_traj[m] = prev_traj[m];
                }
            }
        }
    }

    void TrajPlanner::initialTrajPlanningCheck() {
        // If the agent is disturbed, consider all agents as dynamic obstacles.
        if (is_disturbed) {
            initialTrajPlanningCurrPos();
            initialize_sfc = true;
        }
    }

    void TrajPlanner::goalPlanning() {
        // Timer start
        ros::Time goal_planning_start_time = ros::Time::now();

        if (is_disturbed) {
            agent.current_goal_point = agent.current_state.position;
            return;
        }

        switch (param.goal_mode) {
            case GoalMode::STATIC:
                goalPlanningWithStaticGoal();
                break;
            case GoalMode::RIGHTHAND:
                goalPlanningWithRightHandRule();
                break;
            case GoalMode::PRIORBASED:
                goalPlanningWithPriority();
                break;
            case GoalMode::GRIDBASEDPLANNER:
                goalPlanningWithGridBasedPlanner();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid goal mode");
                break;
        }

        // Timer end
        ros::Time goal_planning_end_time = ros::Time::now();
        statistics.planning_time.goal_planning_time.update((goal_planning_end_time - goal_planning_start_time).toSec());
    }

    void TrajPlanner::goalPlanningWithStaticGoal() {
        agent.current_goal_point = agent.desired_goal_point;
    }

    void TrajPlanner::goalPlanningWithRightHandRule() {
        // If the agent detect deadlock, then change the goal point to the right.
        if (isDeadlock()) {
            point3d z_axis(0, 0, 1);
            agent.current_goal_point = agent.current_state.position +
                                       (agent.desired_goal_point - agent.current_state.position).cross(z_axis);
        } else {
            goalPlanningWithStaticGoal();
        }
    }

    void TrajPlanner::goalPlanningWithPriority() {
        // Find higher priority agents
        std::set<int> high_priority_obstacle_ids;
        high_priority_obstacle_ids.clear();
        int closest_obs_id = -1;
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_point).norm();
        double min_dist_to_obs = SP_INFINITY;
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            if (constraints.isDynamicObstacle(oi)) {
                high_priority_obstacle_ids.emplace(obstacles[oi].id);
                continue;
            }

            if (obstacles[oi].type == ObstacleType::AGENT) {
                point3d obs_goal_position = obstacles[oi].goal_point;
                point3d obs_curr_position = obstacles[oi].position;
                double obs_dist_to_goal = (obs_curr_position - obs_goal_position).norm();
                double dist_to_obs = (obs_curr_position - agent.current_state.position).norm();

                // Do not consider the priority when other agent is near goal.
                if (obs_dist_to_goal < param.goal_threshold) {
                    continue;
                }
                // Do not consider the agents have the same direction
                if (dist_to_goal > param.goal_threshold and
                    (obs_pred_trajs[oi].lastPoint() - obs_pred_trajs[oi].startPoint()).dot(
                            obs_pred_trajs[oi].startPoint() - agent.current_state.position) > 0) {
                    continue;
                }
                // If the agent is near goal, all other agents have higher priority
                // Else the agents with smaller dist_to_goal have higher priority
                if (dist_to_goal < param.goal_threshold or obs_dist_to_goal < dist_to_goal) {
                    if (dist_to_obs < min_dist_to_obs) {
                        min_dist_to_obs = dist_to_obs;
                        closest_obs_id = oi;
                    }
                    high_priority_obstacle_ids.emplace(obstacles[oi].id);
                }
            }
        }

        // If the distance to a higher priority agent is too short, then move away from that agent.
        double priority_dist_threshold = 0.4;
        double dist_keep = priority_dist_threshold + 0.1; //TODO: param need to consider radius of agents!
        if (min_dist_to_obs < priority_dist_threshold) {
            point3d obs_curr_position = obstacles[closest_obs_id].position;
            agent.current_goal_point = agent.current_state.position -
                                          (obs_curr_position - agent.current_state.position).normalized() * dist_keep;
            return;
        }

        // A* considering priority
        bool success = grid_based_planner->planSAPF(agent, distmap_ptr, obstacles, high_priority_obstacle_ids);
        if (not success) {
            // A* without priority
            grid_based_planner->planSAPF(agent, distmap_ptr, obstacles);
        }

        // Find los-free goal from end of the initial trajectory
        point3d los_free_goal = grid_based_planner->findLOSFreeGoal(initial_traj.lastPoint(),
                                                                    agent.desired_goal_point,
                                                                    agent.radius);
        agent.current_goal_point = los_free_goal;
    }

    void TrajPlanner::goalPlanningWithGridBasedPlanner() {
        //update current_goal_point
        GoalOptimizer goal_optimizer(param, mission);
        agent.current_goal_point = goal_optimizer.solve(agent, constraints,
                                                        agent.current_goal_point, agent.next_waypoint);
    }

    void TrajPlanner::constructLSC() {
        // LSC (or BVC) construction
        ros::Time lsc_start_time = ros::Time::now();
        constraints.initializeLSC(obstacles.size());
        if (param.planner_mode == PlannerMode::LSC and param.goal_mode == GoalMode::GRIDBASEDPLANNER){
            generateCLSC();
        } else if (param.planner_mode == PlannerMode::DLSC or param.planner_mode == PlannerMode::LSC) { // RAL 2022
            generateLSC();
        } else if (param.planner_mode == PlannerMode::RECIPROCALRSFC) { // RAL 2021
            generateReciprocalRSFC();
        } else if (param.planner_mode == PlannerMode::BVC) {
            generateBVC();
        } else {
            throw std::invalid_argument("[TrajPlanner] Invalid planner mode");
        }
        ros::Time lsc_end_time = ros::Time::now();
        statistics.planning_time.lsc_generation_time.update((lsc_end_time - lsc_start_time).toSec());
    }

    void TrajPlanner::constructSFC() {
        // SFC construction
        if (param.world_use_octomap) {
            ros::Time sfc_start_time = ros::Time::now();
            generateSFC();
            ros::Time sfc_end_time = ros::Time::now();
            statistics.planning_time.sfc_generation_time.update((sfc_end_time - sfc_start_time).toSec());
        }
    }

    void TrajPlanner::generateReciprocalRSFC() {
        double closest_dist;
        point3d normal_vector;

        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            for (int m = 0; m < param.M; m++) {
                Line obs_path(obs_pred_trajs[oi][m][0], obs_pred_trajs[oi][m][param.n]);
                Line agent_path(initial_traj[m][0], initial_traj[m][param.n]);
                normal_vector = normalVectorBetweenLines(obs_path, agent_path, closest_dist);

                std::vector<double> d;
                d.resize(param.n + 1);
                for (int i = 0; i < param.n + 1; i++) {
                    if (obstacles[oi].type == ObstacleType::AGENT and
                        closest_dist < obs_pred_sizes[oi][m][i] + agent.radius) {
                        d[i] = 0.5 * (obs_pred_sizes[oi][m][i] + agent.radius + closest_dist);
                    } else {
                        d[i] = obs_pred_sizes[oi][m][i] + agent.radius;
                    }
                }

                // downwash
                double downwash = downwashBetween(oi);
                normal_vector.z() = normal_vector.z() / (downwash * downwash);

                constraints.setLSC(oi, m, obs_pred_trajs[oi][m].control_points, normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateLSC() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            // Coordinate transformation
            double downwash = downwashBetween(oi);
            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);

            // Normal vector planning
            // Compute normal vector of LSC
            for (int m = 0; m < param.M; m++) {
                point3d normal_vector;
                if (col_pred_obs_indices.find(oi) != col_pred_obs_indices.end()) {
                    normal_vector = normalVectorDynamicObs(oi, m, downwash);
                } else {
                    normal_vector = normalVectorBetweenPolys(oi, m, initial_traj_trans, obs_pred_traj_trans);
                    if (normal_vector.norm() < SP_EPSILON_FLOAT) {
                        if (obstacles[oi].type == ObstacleType::AGENT) {
                            ROS_WARN("[TrajPlanner] normal_vector is 0");
                        }

                        point3d vector_obs_to_agent = coordinateTransform(
                                agent.current_goal_point - obstacles[oi].position, downwash);
                        normal_vector = vector_obs_to_agent.normalized();
                    }
                }

                // Compute safety margin
                std::vector<double> d;
                d.resize(param.n + 1);
                if (obstacles[oi].type == ObstacleType::AGENT and not constraints.isDynamicObstacle(oi)) {
                    for (int i = 0; i < param.n + 1; i++) {
                        double collision_dist = obstacles[oi].radius + agent.radius;
                        d[i] = 0.5 * (collision_dist +
                                      (initial_traj_trans[m][i] - obs_pred_traj_trans[m][i]).dot(normal_vector));
                    }
                } else {
                    for (int i = 0; i < param.n + 1; i++) {
                        d[i] = obs_pred_sizes[oi][m][i] + agent.radius;
                    }
                }

                // Return to original coordination
                normal_vector.z() = normal_vector.z() / downwash;
                constraints.setLSC(oi, m, obs_pred_trajs[oi][m].control_points, normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateCLSC() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            double collision_dist = obstacles[oi].radius + agent.radius;

            // Coordinate transformation
            double downwash = downwashBetween(oi);
            traj_t initial_traj_trans, obs_pred_traj_trans;
            if(param.world_dimension == 2){
                initial_traj_trans = initial_traj;
                obs_pred_traj_trans = obs_pred_trajs[oi];
            } else {
                initial_traj_trans = initial_traj.coordinateTransform(downwash);
                obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
            }

            // Normal vector planning
            // Compute normal vector of LSC
            for (int m = 0; m < param.M; m++) {
                if(m < param.M - 1){
                    point3d normal_vector = normalVectorBetweenPolys(oi, m, initial_traj_trans, obs_pred_traj_trans);

                    // Compute safety margin
                    std::vector<double> d;
                    d.resize(param.n + 1);
                    for (int i = 0; i < param.n + 1; i++) {
                        d[i] = 0.5 * (collision_dist +
                                      (initial_traj_trans[m][i] - obs_pred_traj_trans[m][i]).dot(normal_vector));
                    }

                    // Return to original coordination
                    normal_vector.z() = normal_vector.z() / downwash;
                    constraints.setLSC(oi, m, obs_pred_trajs[oi][m].control_points, normal_vector, d);
                } else {
                    Line line1(obs_pred_traj_trans.lastPoint(), obstacles[oi].goal_point);
                    Line line2(initial_traj_trans.lastPoint(), agent.current_goal_point);
                    ClosestPoints closest_points = closestPointsBetweenLineSegments(line1, line2);
                    point3d normal_vector = (closest_points.closest_point2 - closest_points.closest_point1).normalized();

                    // Compute safety margin
                    double d = 0.5 * (collision_dist + closest_points.dist);

                    // Return to original coordination
                    normal_vector.z() = normal_vector.z() / downwash;
                    constraints.setLSC(oi, m, closest_points.closest_point1, normal_vector, d);
                }
            }
        }
    }

    void TrajPlanner::generateBVC() {
        // Since the original BVC does not consider downwash, we conduct coordinate transformation to consider downwash.
        point3d normal_vector;
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            // Coordinate transformation
            double downwash = downwashBetween(oi);
            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);

            // normal vector
            normal_vector = (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).normalized();

            // safety margin
            std::vector<double> d;
            d.resize(param.n + 1);
            for (int i = 0; i < param.n + 1; i++) {
                double collision_dist = obstacles[oi].radius + agent.radius;
                d[i] = 0.5 * (collision_dist +
                              (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).dot(normal_vector));
            }

            // Return to original coordination
            normal_vector.z() = normal_vector.z() / downwash;

            for (int m = 0; m < param.M; m++) {
                constraints.setLSC(oi, m, obs_pred_trajs[oi][m].control_points, normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateSFC() {
        if (initialize_sfc) {
            constraints.initializeSFC(agent.current_state.position, agent.radius);
            initialize_sfc = false;
        } else {
            if(param.goal_mode == GoalMode::GRIDBASEDPLANNER) {
                std::vector<point3d> convex_hull;
                convex_hull.emplace_back(initial_traj.lastPoint());
                convex_hull.emplace_back(agent.current_goal_point);
                constraints.constructSFCFromConvexHull(convex_hull, agent.next_waypoint, agent.radius);
                constraints.constructCommunicationRange(agent.next_waypoint);
            } else {
                constraints.constructSFCFromPoint(initial_traj.lastPoint(), agent.current_goal_point, agent.radius);
            }
        }
    }

    traj_t TrajPlanner::trajOptimization() {
        Timer timer;
        TrajOptResult result;

        // Solve QP problem using CPLEX
        timer.reset();
        try {
            result = traj_optimizer->solve(agent, constraints, initial_traj, true);
            if (param.planner_mode == PlannerMode::DLSC and not isSolValid(result)) {
                ROS_WARN("[TrajPlanner] Rerun the solver with default algorithm");
                result = traj_optimizer->solve(agent, constraints, initial_traj, false);
            }
        } catch (...) {
            // Debug
            for (int m = 0; m < param.M; m++) {
                if(param.world_use_octomap) {
                    Box sfc = constraints.getSFC(m);
                    for (const auto &control_point: initial_traj[m].control_points) {
                        bool sfc_check = sfc.isPointInBox(control_point);
                        if (not sfc_check) {
                            ROS_ERROR_STREAM("[TrajPlanner] SFC constraint is not feasible. m: " << m);
                        }
                    }
                }
                for (size_t oi = 0; oi < obstacles.size(); oi++) {
                    for (int i = 0; i < param.n + 1; i++) {
                        LSC lsc = constraints.getLSC(oi, m, i);
                        double margin = (initial_traj[m][i] - lsc.obs_control_point).dot(lsc.normal_vector) - lsc.d;
                        bool lsc_check = margin > 0;
                        if (not lsc_check) {
                            ROS_ERROR_STREAM("[TrajPlanner] LSC constraint is not feasible." <<
                                                                                             " oi: " << oi <<
                                                                                             ", m: " << m <<
                                                                                             ", i: " << i <<
                                                                                             ", margin:" << margin);
                        }
                    }
                }
            }

            //Failsafe
            result.desired_traj = initial_traj;
        }

        timer.stop();
        statistics.planning_time.traj_optimization_time.update(timer.elapsedSeconds());

        return result.desired_traj;
    }

    void TrajPlanner::publishSFC(){
        visualization_msgs::MarkerArray msg_sfc;
        msg_sfc = constraints.convertSFCsToMarkerArrayMsg(mission.color[agent.id], agent.radius);
        pub_sfc.publish(msg_sfc);
    }

    void TrajPlanner::publishLSC(){
        visualization_msgs::MarkerArray msg_lsc;
        msg_lsc = constraints.convertLSCsToMarkerArrayMsg(obstacles, mission.color, agent.radius);
        pub_lsc.publish(msg_lsc);
    }

    void TrajPlanner::publishFeasibleRegion(){
        visualization_msgs::MarkerArray msg_feasible_region;
//        ros::Time start_time = ros::Time::now();
        msg_feasible_region = constraints.feasibleRegionToMarkerArrayMsg(agent.id,
                                                                         mission.color[agent.id]);
//        ros::Time end_time = ros::Time::now();

//        ROS_INFO_STREAM("[TrajPlanner] feasible region gen time: " << (end_time - start_time).toSec());
        pub_feasible_region.publish(msg_feasible_region);
    }

    void TrajPlanner::publishGridPath() {
        visualization_msgs::MarkerArray msg_delete_all = msgDeleteAll();
        pub_grid_path.publish(msg_delete_all);

        visualization_msgs::MarkerArray msg_grid_path_vis =
                grid_based_planner->pathToMarkerMsg(agent.id,
                                                    param.world_frame_id,
                                                    mission.color[agent.id]);
        pub_grid_path.publish(msg_grid_path_vis);
    }

    void TrajPlanner::publishObstaclePrediction() {
        // obstacle prediction vis
        visualization_msgs::MarkerArray msg_obs_pred_traj_vis;
        msg_obs_pred_traj_vis.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = std::to_string(agent.id);

        marker.color.a = 0.07;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        int count = 0;
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            if (obstacles[oi].type == ObstacleType::AGENT) {
                continue;
            }

            double sample_dt = 0.2;
            double N_sample = floor(param.M * param.dt / sample_dt);
            for (int i = 0; i < N_sample; i++) {
                double sample_time = i * sample_dt;
                double obs_pred_size;
                if(i == 0){
                    obs_pred_size = obs_pred_sizes[oi].startPoint();
                }
                else{
                    obs_pred_size = obs_pred_sizes[oi].getPointAt(sample_time);
                }

                marker.scale.x = 2 * obs_pred_size;
                marker.scale.y = 2 * obs_pred_size;
                marker.scale.z = 2 * obs_pred_size * obstacles[oi].downwash;

                if(isnan(obs_pred_size)){
                    ROS_ERROR_STREAM("nan!" << sample_time);
                }

                marker.id = count;
                marker.pose.position = point3DToPointMsg(obs_pred_trajs[oi].getPointAt(sample_time));
                marker.pose.orientation = defaultQuaternion();
                msg_obs_pred_traj_vis.markers.emplace_back(marker);
                count++;
            }
        }
        pub_obs_pred_traj_vis.publish(msg_obs_pred_traj_vis);
    }

    void TrajPlanner::publishGridOccupiedPoints() {
        if (not param.world_use_octomap) {
            return;
        }

        visualization_msgs::MarkerArray msg_delete_all = msgDeleteAll();
        pub_grid_occupied_points.publish(msg_delete_all);

        visualization_msgs::MarkerArray msg_occupied_points =
                grid_based_planner->occupiedPointsToMsg(param.world_frame_id);
        pub_grid_occupied_points.publish(msg_occupied_points);
    }

    bool TrajPlanner::isDeadlock() {
//        double min_safety_ratio = SP_INFINITY;
//        for (int oi = 0; oi < obstacles.size(); oi++) {
//            // Coordinate transformation
//            double downwash = downwashBetween(oi);
//            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
//            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
//            double safety_ratio = (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).norm() / (agent.radius + obstacles[oi].radius);
//            if(min_safety_ratio > safety_ratio){
//                min_safety_ratio = safety_ratio;
//            }
//        }

        //If agent's velocity is lower than some threshold, then it determines agent is in deadlock
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_point).norm();
        return planner_seq > param.deadlock_seq_threshold and
               agent.current_state.velocity.norm() < param.deadlock_velocity_threshold and
               dist_to_goal > 0.2;
//        return min_safety_ratio < 1.1 and dist_to_goal > 0.2;
    }

    bool TrajPlanner::isSolConv() const {
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_point).norm();
        if(planner_seq < param.deadlock_seq_threshold or dist_to_goal < param.goal_threshold){
            return false;
        }

        point3d start_point = prev_traj.startPoint();
        for(int i = 1; i < 3; i++){
            point3d control_point = prev_traj[0].control_points[i];
            double dist = start_point.distance(control_point);
            if(dist > 0.01){ //TODO: param
                return false;
            }
        }

//        ROS_WARN_STREAM("[TrajPlanner] Solution converged, agent" << agent.id);
        return true;
    }

    bool TrajPlanner::isSolConvNearGoal(int closest_agent_idx) const {
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_point).norm();
        if(planner_seq < param.deadlock_seq_threshold or dist_to_goal > param.priority_goal_threshold){
            return false;
        }

        point3d obs_start_point = obstacles[closest_agent_idx].prev_traj.startPoint();
        for(int i = 1; i < 3; i++){
            point3d obs_control_point = obstacles[closest_agent_idx].prev_traj[0].control_points[i];
            double dist = obs_start_point.distance(obs_control_point);
            if(dist > 0.01){ //TODO: param
                return false;
            }
        }

        double downwash = downwashBetween(closest_agent_idx);
        point3d agent_position_trans = coordinateTransform(agent.current_state.position, downwash);
        point3d obs_position_trans = coordinateTransform(obstacles[closest_agent_idx].position, downwash);
        point3d obs_goal_trans = coordinateTransform(obstacles[closest_agent_idx].goal_point, downwash);
        Line obs_line(obs_position_trans, obs_goal_trans);
        ClosestPoints closest_points = closestPointsBetweenPointAndLineSegment(agent_position_trans, obs_line);
        if(closest_points.dist > agent.radius + obstacles[closest_agent_idx].radius){
            return false;
        }

//        ROS_INFO_STREAM("[TrajPlanner] Solution converged near goal, agent" << agent.id);
        return true;
    }

    bool TrajPlanner::isSolConvBySFC() {
        if(not is_sol_converged_by_sfc and isSolConv()){
            is_sol_converged_by_sfc = true;
            sfc_converged = constraints.getSFC(0);
            return true;
        } else if(is_sol_converged_by_sfc){
            if(constraints.getSFC(0) == sfc_converged){
                return true;
            }
            else{
                is_sol_converged_by_sfc = false;
            }
        }

        return false;
    }

    bool TrajPlanner::isSolValid(const TrajOptResult& result) const {
        // Check SFC
        if(param.world_use_octomap){
            for(int m = 0; m < param.M; m++){
                Box sfc = constraints.getSFC(m);
                if(m == 0){
                    for(int i = param.phi; i < param.n + 1; i++){
                        if(not sfc.isPointInBox(result.desired_traj[m][i])) {
                            ROS_WARN("[TrajPlanner] solution is not valid due to SFC");
                            return false;
                        }
                    }
                } else {
                    if(not sfc.isSegmentInBox(result.desired_traj[m])) {
                        ROS_WARN("[TrajPlanner] solution not valid due to SFC");
                        return false;
                    }
                }
            }
        }


        // Check LSC
//        for(int oi = 0; oi < obstacles.size(); oi++) {
//            for (int m = 0; m < param.M; m++) {
//                for (int i = 0; i < param.n + 1; i++) {
//                    if (m == 0 and i < param.phi) {
//                        continue;
//                    }
//
//                    LSC lsc = constraints.getLSC(oi, m, i);
//                    if (lsc.normal_vector.dot(result.desired_traj[m][i] - lsc.obs_control_point) - lsc.d <
//                        -SP_EPSILON_FLOAT) {
//                        ROS_WARN("[TrajPlanner] solution is not valid due to LSC");
//                        return false;
//                    }
//                }
//            }
//        }

        // Check dynamical limit
        double dyn_err_tol_ratio = 0.01;
        State state = result.desired_traj.getStateAt(param.multisim_time_step);
        for(int k = 0; k < param.world_dimension; k++){
            if(abs(state.velocity(k)) > agent.max_vel[k] * (1 + dyn_err_tol_ratio)) {
                ROS_WARN("[TrajPlanner] solution is not valid due to max_vel");
                return false;
            }
            if(abs(state.acceleration(k)) > agent.max_acc[k] * (1 + dyn_err_tol_ratio)) {
                ROS_WARN("[TrajPlanner] solution is not valid due to max_acc");
                return false;
            }
        }

        return true;
    }

    int TrajPlanner::findObstacleIdxByObsId(int obs_id) const {
        int oi = -1;
        for (size_t i = 0; i < obstacles.size(); i++) {
            if (obstacles[i].id == obs_id) {
                oi = i;
            }
        }
        return oi;
    }

    double TrajPlanner::distanceToGoalByObsId(int obs_id) const {
        int obs_idx = findObstacleIdxByObsId(obs_id);
        return distanceToGoalByObsIdx(obs_idx);
    }

    double TrajPlanner::distanceToGoalByObsIdx(int obs_idx) const {
        return obstacles[obs_idx].goal_point.distance(obstacles[obs_idx].position);
    }

    double TrajPlanner::computeCollisionTimeToDistmap(const point3d &start_position,
                                                      const point3d &goal_position,
                                                      double agent_radius,
                                                      double time_horizon) {
        double collision_time = 0;
        bool isCollided = distmap_ptr->getDistance(start_position) < agent_radius;
        if (goal_position == start_position) {
            if (isCollided) {
                collision_time = 0;
            } else {
                collision_time = SP_INFINITY;
            }
            return collision_time;
        }

        double search_time_step = 0.1;
        double current_time = 0;
        point3d current_search_point;
        while (!isCollided and current_time < time_horizon) {
            current_time += search_time_step;
            current_search_point = start_position + (goal_position - start_position) * (current_time / time_horizon);
            isCollided = distmap_ptr->getDistance(current_search_point) < agent_radius;
        }

        if (isCollided) {
            collision_time = current_time;
        } else {
            collision_time = SP_INFINITY;
        }

        return collision_time;
    }

//    double TrajPlanner::computeMinCollisionTime() {
//        double collision_time, min_collision_time = SP_INFINITY;
//        double total_time_horizon = M * param.dt;
//
//        if (param.world_use_octomap) {
//            collision_time = computeCollisionTimeToDistmap(initial_traj.startPoint(),
//                                                           initial_traj.lastPoint(),
//                                                           agent.radius,
//                                                           total_time_horizon);
//            if (min_collision_time > collision_time) {
//                min_collision_time = collision_time;
//            }
//        }
//
//        size_t N_obs = obstacles.size();
//        for (int oi = 0; oi < N_obs; oi++) {
//            collision_time = computeCollisionTime(obs_pred_trajs[oi].startPoint(),
//                                                  obs_pred_trajs[oi].lastPoint(),
//                                                  initial_traj.startPoint(),
//                                                  initial_traj.lastPoint(),
//                                                  obstacles[oi].radius + agent.radius,
//                                                  total_time_horizon);
//
//            if (min_collision_time > collision_time) {
//                min_collision_time = collision_time;
//            }
//
//            if (param.world_use_octomap) {
//                collision_time = computeCollisionTimeToDistmap(obs_pred_trajs[oi].startPoint(),
//                                                               obs_pred_trajs[oi].lastPoint(),
//                                                               obstacles[oi].radius,
//                                                               total_time_horizon);
//                if (min_collision_time > collision_time) {
//                    min_collision_time = collision_time;
//                }
//            }
//
//            for (int oj = 0; oj < N_obs; oj++) {
//                if (oj > oi) {
//                    collision_time = computeCollisionTime(obs_pred_trajs[oi].startPoint(),
//                                                          obs_pred_trajs[oi].lastPoint(),
//                                                          obs_pred_trajs[oj].startPoint(),
//                                                          obs_pred_trajs[oj].lastPoint(),
//                                                          obstacles[oi].radius + obstacles[oj].radius,
//                                                          total_time_horizon);
//                } else {
//                    continue;
//                }
//
//                if (min_collision_time > collision_time) {
//                    min_collision_time = collision_time;
//                }
//            }
//        }
//
//        return min_collision_time;
//    }

    point3d TrajPlanner::normalVectorBetweenLines(const Line &obs_path, const Line &agent_path, double &closest_dist) {
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_path, agent_path);
        closest_dist = closest_points.dist;

        point3d delta, normal_vector;
        delta = closest_points.closest_point2 - closest_points.closest_point1;
        normal_vector = delta.normalized();
        if (normal_vector.norm() == 0) {
            ROS_WARN("[Util] heuristic method was used to get normal vector");
            point3d a, b;
            a = agent_path.start_point - obs_path.start_point;
            b = agent_path.end_point - obs_path.end_point;
            if (a.norm() == 0 and b.norm() == 0) {
                normal_vector = point3d(1, 0, 0);
            } else {
                normal_vector = (b - a).cross(point3d(0, 0, 1));
            }
        }
        return normal_vector;
    }

    point3d TrajPlanner::normalVectorBetweenPolys(int oi, int m,
                                                  const traj_t &initial_traj_trans,
                                                  const traj_t &obs_pred_traj_trans) {
        size_t n_control_points = param.n + 1;
        points_t control_points_rel;
        control_points_rel.resize(n_control_points);
        for (size_t i = 0; i < n_control_points; i++) {
            control_points_rel[i] = initial_traj_trans[m][i] - obs_pred_traj_trans[m][i];

            if (obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE and
                obstacles[oi].downwash > param.obs_downwash_threshold) {
                control_points_rel[i].z() = 0;
            }
        }

        ClosestPoints closest_points = closestPointsBetweenPointAndConvexHull(point3d(0, 0, 0),
                                                                              control_points_rel);
        point3d normal_vector = closest_points.closest_point2.normalized();

        if (obstacles[oi].type == AGENT and closest_points.dist < agent.radius + obstacles[oi].radius - 0.001) {
            ROS_WARN_STREAM("[TrajPlanner] invalid normal_vector: " << normal_vector
                                                                    << ", dist: " << closest_points.dist
                                                                    << ", agent_id: " << agent.id << ", obs_id: "
                                                                    << obstacles[oi].id);
        }
        return normal_vector;
    }

    point3d TrajPlanner::normalVectorDynamicObs(int oi, int m, double downwash) {
        point3d normal_vector;

        //Coordinate transformation
        point3d vector_obs_to_goal = coordinateTransform(agent.current_goal_point - obstacles[oi].position,
                                                         downwash);
        point3d vector_obs_to_agent = coordinateTransform(agent.current_state.position - obstacles[oi].position,
                                                          downwash);
        if (obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE and
            obstacles[oi].downwash > param.obs_downwash_threshold) {
            vector_obs_to_goal.z() = 0;
            vector_obs_to_agent.z() = 0;
        }

        Line obs_path(obs_pred_trajs[oi][m][0], obs_pred_trajs[oi][m][param.n]);
        Line agent_path(initial_traj[m][0], initial_traj[m][param.n]);
        double closest_dist = 0;
        normal_vector = normalVectorBetweenLines(obs_path, agent_path, closest_dist);

        return normal_vector;
    }

    double TrajPlanner::downwashBetween(int oi) const {
        double downwash = 1;
        if (obstacles[oi].type == ObstacleType::AGENT) {
            downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                       (agent.radius + obstacles[oi].radius);
        } else {
            downwash = (agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                       (agent.radius + obstacles[oi].radius);
        }

        return downwash;
    }

    double TrajPlanner::downwashBetween(int oi, int oj) const {
        double downwash = 1;

        if (obstacles[oi].type != obstacles[oj].type) {
            if (obstacles[oi].type == ObstacleType::AGENT) {
                downwash = (obstacles[oi].radius + obstacles[oj].downwash * obstacles[oj].radius) /
                           (obstacles[oi].radius + obstacles[oj].radius);
            } else if (obstacles[oj].type == ObstacleType::AGENT) {
                downwash = (obstacles[oi].downwash * obstacles[oi].radius + obstacles[oj].radius) /
                           (obstacles[oi].radius + obstacles[oj].radius);
            }
        } else {
            downwash = (obstacles[oi].downwash * obstacles[oi].radius +
                        obstacles[oj].downwash * obstacles[oj].radius) /
                       (obstacles[oi].radius + obstacles[oj].radius);
        }

        return downwash;
    }

    point3d TrajPlanner::coordinateTransform(const point3d &point, double downwash) {
        point3d point_trans = point;
        point_trans.z() = point_trans.z() / downwash;
        return point_trans;
    }

    double TrajPlanner::computePriority(double dist_to_goal, double goal_threshold){
        double priority;
        if(dist_to_goal < goal_threshold){
            priority = dist_to_goal;
        }
        else{
            priority = goal_threshold + 1/dist_to_goal;
        }

        return priority;
    }

    void TrajPlanner::updateCurrentGoal(){
        for(int m = param.M - 2; m >= 0; m--){
            Box sfc_next = constraints.getSFC(m + 1);
            Box sfc_curr = constraints.getSFC(m);
            Box inter_sfc = sfc_next.intersection(sfc_curr);
            Segment<point3d> target_segment = initial_traj[m];
            point3d last_point = target_segment.lastPoint();
            if(inter_sfc.isPointInBox(last_point) and
               not sfc_curr.isPointInBoxStrictly(last_point) and
               not inter_sfc.isSegmentInBox(target_segment)){
                agent.current_goal_point = target_segment.lastPoint();
            }
        }
    }
}