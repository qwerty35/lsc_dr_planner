#include "goal_optimizer.hpp"

namespace DynamicPlanning {
    GoalOptimizer::GoalOptimizer(const Param &_param, const Mission &_mission)
            : param(_param), mission(_mission) {}

    point3d GoalOptimizer::solve(const Agent& agent,
                                 const CollisionConstraints& constraints,
                                 const point3d &current_goal_point,
                                 const point3d &next_waypoint) {

        if(current_goal_point.distance(next_waypoint) < SP_EPSILON_FLOAT){
            return next_waypoint;
        }

        point3d goal;
        IloEnv env;
        IloCplex cplex(env);
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);

        // Set CPLEX parameters
        cplex.setParam(IloCplex::Param::Threads, 6);
//        cplex.setParam(IloCplex::Param::TimeLimit, 0.1); // For time limit

        // Set CPLEX algorithm
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Algorithm::Primal);
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Algorithm::Dual);

        // Initialize QP model
        populatebyrow(model, var, con, agent, constraints, current_goal_point, next_waypoint);
        cplex.extract(model);

        std::string QPmodel_path = param.package_path + "/log/QPmodel_goalOpt.lp";
        std::string conflict_path = param.package_path + "/log/conflict_goalOpt.lp";
        if (param.log_solver) {
            cplex.exportModel(QPmodel_path.c_str());
        } else {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }

        // Solve QP
        try {
            IloBool success = cplex.solve();

            // Desired trajectory
            IloNumArray vals(env);
            cplex.getValues(vals, var);
            goal = (current_goal_point - next_waypoint) * vals[0] + next_waypoint;
            env.end();
        }
        catch (IloException &e) {
            cplex.exportModel(QPmodel_path.c_str());
            if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
                (cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded)) {
                ROS_ERROR_STREAM(
                        "[GoalOptimizer] CPLEX No solution at mav " << agent.id << ", starting Conflict refinement");
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
            ROS_ERROR_STREAM("[GoalOptimizer] CPLEX Unknown exception caught at iteration ");
            if (not param.log_solver) {
                cplex.exportModel(QPmodel_path.c_str());
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }

        return goal;
    }

    void GoalOptimizer::populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                                      const Agent &agent, const CollisionConstraints &constraints,
                                      const point3d &current_goal_point,
                                      const point3d &next_waypoint) {
        size_t N_obs = constraints.getObsSize();

        IloEnv env = model.getEnv();
        // Initialize control points variables
        std::string name = "t";
        x.add(IloNumVar(env, 0, 1 + SP_EPSILON_FLOAT));
//        x.add(IloNumVar(env, 0, IloInfinity));
        x[0].setName(name.c_str());

        // Cost function - distance to next waypoint
        IloNumExpr cost(env);
        cost += x[0];
        model.add(IloMinimize(env, cost));

        // Inequality Constraints
        // SFC
        if (param.world_use_octomap) {
            std::vector<LSC> lscs = constraints.getSFC(param.M - 1).convertToLSCs(param.world_dimension);
            for (const auto &lsc: lscs) {
                IloNumExpr expr(env);

                for (int k = 0; k < param.world_dimension; k++) {
                    expr += lsc.normal_vector(k) *
                            ((current_goal_point(k) - next_waypoint(k)) * x[0] + next_waypoint(k) -
                             lsc.obs_control_point(k));
                }
                expr += -lsc.d;

                c.add(expr >= 0);
                expr.end();
            }
        }

        // LSC or BVC
        for (size_t oi = 0; oi < N_obs; oi++) {
            LSC lsc = constraints.getLSC(oi, param.M - 1, param.n);
            if (lsc.normal_vector.norm() < SP_EPSILON_FLOAT) {
                continue;
            }

            IloNumExpr expr(env);
            for (int k = 0; k < param.world_dimension; k++) {
                expr += lsc.normal_vector(k) * ((current_goal_point(k) - next_waypoint(k)) * x[0] + next_waypoint(k) -
                                                lsc.obs_control_point(k));
            }
            expr += -lsc.d;

            c.add(expr >= 0);
            expr.end();
        }

        model.add(c);
    }
}