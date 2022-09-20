#ifndef LSC_PLANNER_GOAL_OPTIMIZER_HPP
#define LSC_PLANNER_GOAL_OPTIMIZER_HPP

#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <collision_constraints.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

namespace DynamicPlanning {
    class GoalOptimizer {
    public:
        GoalOptimizer(const Param& param, const Mission& mission);

        point3d solve(const Agent& agent,
                      const CollisionConstraints& constraints,
                      const point3d &current_goal_point,
                      const point3d &next_waypoint);

    private:
        Param param;
        Mission mission;

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                           const Agent& agent, const CollisionConstraints& constraints,
                           const point3d &current_goal_point,
                           const point3d &next_waypoint);
    };
}


#endif //LSC_PLANNER_GOAL_OPTIMIZER_HPP
