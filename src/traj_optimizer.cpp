#include "traj_optimizer.hpp"

namespace DynamicPlanning {
    TrajOptimizer::TrajOptimizer(const Param &_param, const Mission &_mission, const Eigen::MatrixXd &_B)
            : param(_param), mission(_mission), B(_B) {
        // Initialize trajectory param, offsets
        dim = param.world_dimension;
        M = param.M;
        n = param.n;
        phi = param.phi;
        dt = param.dt;

        // Build constraint matrices
        buildQBase();
        buildAeqBase();
    }

    TrajOptResult TrajOptimizer::solve(const Agent& agent,
                                       const CollisionConstraints& constraints,
                                       const traj_t& initial_traj,
                                       bool use_primal_algorithm) {
        TrajOptResult result;
        result.desired_traj = Trajectory<point3d>(M, n, dt);

        IloEnv env;
        IloCplex cplex(env);
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);

        // Set CPLEX parameters
        cplex.setParam(IloCplex::Param::Threads, 6);
//        cplex.setParam(IloCplex::Param::TimeLimit, 0.1); // For time limit

        // Set CPLEX algorithm
        if(use_primal_algorithm){
            cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Algorithm::Primal);
//            cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Algorithm::Dual);
        }

        // Initialize QP model
        populatebyrow(model, var, con, agent, constraints, initial_traj);
        cplex.extract(model);

        std::string QPmodel_path = param.package_path + "/log/QPmodel_trajOpt.lp";
        std::string conflict_path = param.package_path + "/log/conflict_trajOpt.lp";
        if (param.log_solver) {
            cplex.exportModel(QPmodel_path.c_str());
        } else {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }

        // Solve QP
        int offset_seg = n + 1;
        int offset_dim = M * (n + 1);
        int offset_slack_cont = dim * M * (n + 1);
        int offset_slack_col;
        if (param.slack_mode == SlackMode::CONTINUITY) {
            offset_slack_col = dim * M * (n + 1) + dim * 4;
        } else {
            offset_slack_col = offset_slack_cont;
        }

        try {
            IloBool success = cplex.solve();

            // Desired trajectory
            IloNumArray vals(env);
            cplex.getValues(vals, var);
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    if (dim == 3) {
                        result.desired_traj[m][i] = point3d(vals[0 * offset_dim + m * offset_seg + i],
                                                            vals[1 * offset_dim + m * offset_seg + i],
                                                            vals[2 * offset_dim + m * offset_seg + i]);
                    } else {
                        result.desired_traj[m][i] = point3d(vals[0 * offset_dim + m * offset_seg + i],
                                                            vals[1 * offset_dim + m * offset_seg + i],
                                                            param.world_z_2d);
                    }
                }
            }

            // Collision predicted obstacles
            int obs_slack_idx = 0;
            size_t N_obs = constraints.getObsSize();
            for (size_t oi = 0; oi < N_obs; oi++) {
                if (param.slack_mode == SlackMode::COLLISIONCONSTRAINT or constraints.isDynamicObstacle(oi)) {
                    double slack_cost = 0;
                    for (int m = 0; m < M; m++) {
                        slack_cost += abs(vals[offset_slack_col + M * obs_slack_idx + m]);
                    }

                    obs_slack_idx++;
                }
            }

            // Total QP cost
            result.total_qp_cost = cplex.getObjValue();
            env.end();
        }
        catch (IloException &e) {
            cplex.exportModel(QPmodel_path.c_str());
            if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
                (cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded)) {
                ROS_ERROR_STREAM(
                        "[TrajOptimizer] CPLEX No solution at mav " << agent.id << ", starting Conflict refinement");
                IloConstraintArray infeas(env);
                IloNumArray preferences(env);

                infeas.add(con);
                for (IloInt i = 0; i < var.getSize(); i++) {
                    if (var[i].getType() != IloNumVar::Bool) {
                        infeas.add(IloBound(var[i], IloBound::Lower));
                        infeas.add(IloBound(var[i], IloBound::Upper));
                    }
                }

                for (IloInt i = 0; i < infeas.getSize(); i++) {
                    preferences.add(1.0);  // User may wish to assign unique preferences
                }

                if (cplex.refineConflict(infeas, preferences)) {
                    IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
                    env.getImpl()->useDetailedDisplay(IloTrue);
                    std::cout << "Conflict :" << std::endl;
                    for (IloInt i = 0; i < infeas.getSize(); i++) {
                        if (conflict[i] == IloCplex::ConflictMember)
                            std::cout << "Proved  : c" << i << infeas[i] << std::endl;
                        if (conflict[i] == IloCplex::ConflictPossibleMember)
                            std::cout << "Possible: c" << i << infeas[i] << std::endl;
                    }
                    cplex.writeConflict(conflict_path.c_str());
                } else {
                    ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Conflict could not be refined");
                }
            } else {
                ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Concert exception caught: " << e);
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }
        catch (...) {
            ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Unknown exception caught at iteration ");
            if (not param.log_solver) {
                cplex.exportModel(QPmodel_path.c_str());
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }

        return result;
    }

    void TrajOptimizer::updateParam(const Param &_param) {
        param = _param;
    }

    // Cost matrix Q
    void TrajOptimizer::buildQBase() {
        Q_base = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for (int k = phi; k > phi - param.phi_n; k--) {
            Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(n + 1, n + 1);
            for (int i = 0; i < n + 1; i++) {
                for (int j = 0; j < n + 1; j++) {
                    if (i + j - 2 * k + 1 > 0)
                        Z(i, j) =
                                (double) coef_derivative(i, k) * coef_derivative(j, k) / (i + j - 2 * k + 1);
                }
            }
            Z = B * Z * B.transpose();
            Z = Z * pow(dt, -2 * k + 1);
            Q_base += Z;
        }
    }

    void TrajOptimizer::buildAeqBase() {
        // Build A_0, A_T
        Eigen::MatrixXd A_0 = Eigen::MatrixXd::Zero(n + 1, n + 1);
        Eigen::MatrixXd A_T = Eigen::MatrixXd::Zero(n + 1, n + 1);
        if (n == 5 and phi == 3) {
            A_0 << 1, 0, 0, 0, 0, 0,
                    -1, 1, 0, 0, 0, 0,
                    1, -2, 1, 0, 0, 0,
                    -1, 3, -3, 1, 0, 0,
                    1, -4, 6, -4, 1, 0,
                    -1, 5, -10, 10, -5, 1;

            A_T << 0, 0, 0, 0, 0, 1,
                    0, 0, 0, 0, -1, 1,
                    0, 0, 0, 1, -2, 1,
                    0, 0, -1, 3, -3, 1,
                    0, 1, -4, 6, -4, 1,
                    -1, 5, -10, 10, -5, 1;
        } else {
            //TODO: Compute A_0, A_T when n is not 5
            throw std::invalid_argument("[TrajOptimizer] Currently, only n=5, phi=3 is available");
        }

        Aeq_base = Eigen::MatrixXd::Zero((M - 2) * phi, M * (n + 1));
        for (int m = 2; m < M; m++) {
            int nn = 1;
            for (int j = 0; j < phi; j++) {
                Aeq_base.block(phi * (m - 2) + j, (n + 1) * (m - 1), 1, n + 1) =
                        pow(dt, -j) * nn * A_T.row(j);
                Aeq_base.block(phi * (m - 2) + j, (n + 1) * m, 1, n + 1) =
                        -pow(dt, -j) * nn * A_0.row(j);
                nn = nn * (n - j);
            }
        }
    }

    void TrajOptimizer::populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                                      const Agent &agent, const CollisionConstraints &constraints,
                                      const traj_t &initial_traj) {
        size_t N_obs = constraints.getObsSize();
        int offset_seg = n + 1;
        int offset_dim = M * (n + 1);
        int offset_slack_cont_front_vel = dim * M * (n + 1);
        int offset_slack_cont_front_acc = dim * M * (n + 1) + dim * 1;
        int offset_slack_cont_back_vel = dim * M * (n + 1) + dim * 2;
        int offset_slack_cont_back_acc = dim * M * (n + 1) + dim * 3;
        int offset_slack_col;
        if (param.slack_mode == SlackMode::CONTINUITY) {
            offset_slack_col = dim * M * (n + 1) + dim * 4;
        } else {
            offset_slack_col = dim * M * (n + 1);
        }


        IloEnv env = model.getEnv();
        // Initialize control points variables
        std::string name;
        double lower_bound, upper_bound;
        for (int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    int row = k * offset_dim + m * offset_seg + i;
                    if (k == 0) {
                        name = "x_" + std::to_string(m) + "_" + std::to_string(i);
                    } else if (k == 1) {
                        name = "y_" + std::to_string(m) + "_" + std::to_string(i);
                    } else if (k == 2) {
                        name = "z_" + std::to_string(m) + "_" + std::to_string(i);
                    } else {
                        throw std::invalid_argument("[TrajOptimizer] Invalid output dimension, output_dim > 3");
                    }

                    lower_bound = mission.world_min(k);
                    upper_bound = mission.world_max(k);

                    if(k == 2 and m == 0 and param.planner_mode == PlannerMode::RECIPROCALRSFC){ // To avoid numerical error
                        lower_bound = -100;
                        upper_bound = 100;
                    }

                    if (m == 0 and i < 3) {
                        // Do not adjust the constraint at the initial state
                        x.add(IloNumVar(env, -IloInfinity, IloInfinity));
                    } else {
                        x.add(IloNumVar(env, lower_bound, upper_bound));
                    }

                    x[row].setName(name.c_str());
                }
            }
        }

        int obs_slack_idx = 0;
        for (size_t oi = 0; oi < N_obs; oi++) {
            if (param.slack_mode == SlackMode::COLLISIONCONSTRAINT or constraints.isDynamicObstacle(oi)) {
                for (int m = 0; m < M; m++) {
                    x.add(IloNumVar(env, -IloInfinity, 0));
                    int row = offset_slack_col + M * obs_slack_idx + m;
                    name = "epsilon_slack_col_" + std::to_string(oi) + "_" + std::to_string(m);
                    x[row].setName(name.c_str());
                }
                obs_slack_idx++;
            }
        }

        // Cost function - 1. jerk
        IloNumExpr cost(env);
        for (int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    int row = k * offset_dim + m * offset_seg + i;
                    for (int j = 0; j < n + 1; j++) {
                        int col = k * offset_dim + m * offset_seg + j;
                        if (Q_base(i, j) != 0 and param.control_input_weight != 0) {
                            cost += param.control_input_weight * Q_base(i, j) * x[row] * x[col];
                        }
                    }
                }
            }
        }

        // Cost function - 2. error to goal
        int terminal_segments = getTerminalSegments_old(agent);
        for (int m = M - terminal_segments; m < M; m++) {
            cost += param.terminal_weight *
                    (x[0 * offset_dim + m * offset_seg + n] - agent.current_goal_point.x()) *
                    (x[0 * offset_dim + m * offset_seg + n] - agent.current_goal_point.x());
            cost += param.terminal_weight *
                    (x[1 * offset_dim + m * offset_seg + n] - agent.current_goal_point.y()) *
                    (x[1 * offset_dim + m * offset_seg + n] - agent.current_goal_point.y());
            if (dim == 3) {
                cost += param.terminal_weight *
                        (x[2 * offset_dim + m * offset_seg + n] - agent.current_goal_point.z()) *
                        (x[2 * offset_dim + m * offset_seg + n] - agent.current_goal_point.z());
            }
        }
        model.add(IloMinimize(env, cost));

        // Continuity constraints
        for (int k = 0; k < dim; k++) {
            // Front position
            c.add(x[k * offset_dim + 0 * offset_seg + 0] == agent.current_state.position(k));

            // Back position
            if(M > 1){
                c.add(x[k * offset_dim + 0 * offset_seg + n] - x[k * offset_dim + 1 * offset_seg + 0] == 0);
            }


            // Front velocity
            c.add(pow(dt, -1) * n *
                  (x[k * offset_dim + 0 * offset_seg + 1] - x[k * offset_dim + 0 * offset_seg + 0])
                  == agent.current_state.velocity(k));

            // Front acceleration
            c.add(pow(dt, -2) * n * (n - 1) *
                  (x[k * offset_dim + 0 * offset_seg + 2] -
                   2 * x[k * offset_dim + 0 * offset_seg + 1] +
                   x[k * offset_dim + 0 * offset_seg + 0]) == agent.current_state.acceleration(k));

            // Back velocity
            c.add((x[k * offset_dim + 1 * offset_seg + 1] -
                   x[k * offset_dim + 1 * offset_seg + 0]) -
                  (x[k * offset_dim + 0 * offset_seg + n] -
                   x[k * offset_dim + 0 * offset_seg + n - 1]) == 0);

            // Back acceleration
            c.add((x[k * offset_dim + 1 * offset_seg + 2] -
                   2 * x[k * offset_dim + 1 * offset_seg + 1] +
                   x[k * offset_dim + 1 * offset_seg + 0]) -
                  (x[k * offset_dim + 0 * offset_seg + n] -
                   2 * x[k * offset_dim + 0 * offset_seg + n - 1] +
                   x[k * offset_dim + 0 * offset_seg + n - 2]) == 0);
        }


        // Continuity constraints
        for (int k = 0; k < dim; k++) {
            for (int i = 0; i < (M - 2) * phi; i++) {
                IloNumExpr expr(env);
                for (int j = 0; j < offset_dim; j++) {
                    if (Aeq_base(i, j) != 0) {
                        expr += Aeq_base(i, j) * x[k * offset_dim + j];
                    }
                }
                c.add(expr == 0);
                expr.end();
            }
        }

        // Inequality Constraints
        // SFC
        if (param.world_use_octomap) {
            for (int m = 0; m < M; m++) {
                std::vector<LSC> lscs = constraints.getSFC(m).convertToLSCs(param.world_dimension);
                for (const auto &lsc: lscs) {
                    for (int j = 0; j < n + 1; j++) {
                        if (m == 0 and j < phi) {
                            continue; // Do not adjust constraint at initial state
                        }

                        IloNumExpr expr(env);
                        expr += lsc.normal_vector.x() *
                                (x[0 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.x());
                        expr += lsc.normal_vector.y() *
                                (x[1 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.y());
                        if (dim == 3) {
                            expr += lsc.normal_vector.z() *
                                    (x[2 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.z());
                        }
                        expr += -lsc.d;

                        c.add(expr >= 0);
                        expr.end();
                    }
                }
            }
        }

        // LSC or BVC
        obs_slack_idx = 0;
        for (int oi = 0; oi < N_obs; oi++) {
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    if (m == 0 and i < phi) {
                        continue; // Do not adjust constraint at initial state
                    }

                    LSC lsc = constraints.getLSC(oi, m, i);
                    if(lsc.normal_vector.norm() < SP_EPSILON_FLOAT){
                        continue;
                    }

                    IloNumExpr expr(env);
                    expr += lsc.normal_vector.x() *
                            (x[0 * offset_dim + m * offset_seg + i] - lsc.obs_control_point.x());
                    expr += lsc.normal_vector.y() *
                            (x[1 * offset_dim + m * offset_seg + i] - lsc.obs_control_point.y());
                    if (dim == 3) {
                        expr += lsc.normal_vector.z() *
                                (x[2 * offset_dim + m * offset_seg + i] - lsc.obs_control_point.z());
                    }

                    if (param.slack_mode == SlackMode::COLLISIONCONSTRAINT or constraints.isDynamicObstacle(oi)) {
                        expr += -(lsc.d + x[offset_slack_col + M * obs_slack_idx + m]);
                    } else {
                        expr += -lsc.d;
                    }

                    c.add(expr >= 0);
                    expr.end();
                }
            }

            if (param.slack_mode == SlackMode::COLLISIONCONSTRAINT or constraints.isDynamicObstacle(oi)) {
                obs_slack_idx++;
            }
        }

        // Dynamic feasibility
        for (int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                // Maximum velocity
                for (int i = 0; i < n; i++) {
                    if (m == 0 and (i == 0 or i == 1)) {
                        continue; //Do not adjust constraint at the initial state
                    }

                    c.add(pow(dt, -1) * n *
                          (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) <=
                          agent.max_vel[k]);
                    c.add(-pow(dt, -1) * n *
                          (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) <=
                          agent.max_vel[k]);
                }

                // Maximum acceleration
                for (int i = 0; i < n - 1; i++) {
                    if (m == 0 and i == 0) {
                        continue; //Do not adjust constraint at initial state
                    }

                    c.add(pow(dt, -2) * n * (n - 1) *
                          (x[k * offset_dim + m * offset_seg + i + 2] -
                           2 * x[k * offset_dim + m * offset_seg + i + 1] +
                           x[k * offset_dim + m * offset_seg + i]) <=
                          agent.max_acc[k]);
                    c.add(-pow(dt, -2) * n * (n - 1) *
                          (x[k * offset_dim + m * offset_seg + i + 2] -
                           2 * x[k * offset_dim + m * offset_seg + i + 1] +
                           x[k * offset_dim + m * offset_seg + i]) <=
                          agent.max_acc[k]);
                }
            }
        }


        // Communication range
        if (param.communication_range > 0) {
            for(int k = 0; k < dim; k++) {
                for (int mi = 0; mi < M; mi++) {
                    for (int m = mi; m < M; m++) {
                        c.add(x[k * offset_dim + m * offset_seg + n] -
                              x[k * offset_dim + mi * offset_seg + 0] <=
                              0.5 * param.communication_range - agent.radius);
                        c.add(-x[k * offset_dim + m * offset_seg + n] +
                              x[k * offset_dim + mi * offset_seg + 0] <=
                              0.5 * param.communication_range - agent.radius);
                    }
                }
            }

            for(int k = 0; k < dim; k++) {
                for (int m = 0; m < M; m++) {
                    c.add(x[k * offset_dim + m * offset_seg + n] - agent.next_waypoint(k) <=
                          0.5 * param.communication_range - SP_EPSILON_FLOAT);
                    c.add(-x[k * offset_dim + m * offset_seg + n] + agent.next_waypoint(k) <=
                          0.5 * param.communication_range - SP_EPSILON_FLOAT);
                }
            }
        }

        // Additional constraints for feasible LSC
        // Stop at the end of planning horizon
        if (param.planner_mode == PlannerMode::LSC) {
            for (int k = 0; k < dim; k++) {
                int m = M - 1;
                for (int i = 1; i < phi; i++) {
                    c.add(x[k * offset_dim + m * offset_seg + n] - x[k * offset_dim + m * offset_seg + n - i] == 0);
                }
            }
        }

        model.add(c);
    }

    int TrajOptimizer::getTerminalSegments(const Agent &agent, const traj_t &initial_traj) const {
        for (int m = 0; m < M; m++) {
            if (agent.current_goal_point.distance(initial_traj[m].control_points.back()) < param.goal_threshold) {
                if(m == 0){
                    return M;
                } else {
                    return M - m + 1;
                }
            }
        }

        return 1;
    }

    int TrajOptimizer::getTerminalSegments_old(const Agent& agent) const{
        int terminal_segments;
        double ideal_flight_time =
                (agent.current_goal_point - agent.current_state.position).norm() / agent.nominal_velocity;
        terminal_segments = std::max(static_cast<int>((M * param.dt - ideal_flight_time + SP_EPSILON) / param.dt),
                                     1); //increase terminal segments near goal point
//            terminal_segments = std::min(M, 3);
        return terminal_segments;
    }
}