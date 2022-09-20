#ifndef LSC_PLANNER_TRAJ_OPTIMIZER_HPP
#define LSC_PLANNER_TRAJ_OPTIMIZER_HPP

#include <sp_const.hpp>
#include <polynomial.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <collision_constraints.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

namespace DynamicPlanning {
    struct TrajOptResult{
        traj_t desired_traj;
        double total_qp_cost = 0;
    };

    class TrajOptimizer {
    public:
        TrajOptimizer(const Param& param, const Mission& mission, const Eigen::MatrixXd& B);

        TrajOptResult solve(const Agent& agent, const CollisionConstraints& constraints,
                            const traj_t& initial_traj, bool use_primal_algorithm);

        void updateParam(const Param& param);

    private:
        Param param;
        Mission mission;
        Eigen::MatrixXd Q_base, Aeq_base, B;

        // Frequently used constants
        int M, n, phi, dim;
        double dt;

        // Cost matrix Q
        void buildQBase();

        // Constraint matrix A_eq x > d_eq
        void buildAeqBase();

//        void buildDeq(const Agent& agent);

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                           const Agent& agent, const CollisionConstraints& constraints, const traj_t& initial_traj);

        [[nodiscard]] int getTerminalSegments(const Agent& agent, const traj_t& initial_traj) const;

        [[nodiscard]] int getTerminalSegments_old(const Agent& agent) const;
    };
}


#endif //LSC_PLANNER_TRAJ_OPTIMIZER_HPP
